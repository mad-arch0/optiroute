import os
import time
import threading
from pathlib import Path
from typing import List, Optional

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

try:
    import serial
except ImportError:
    serial = None

BASE_DIR = Path(__file__).resolve().parent
WEB_DIR = BASE_DIR / "web"

SERIAL_PORT = os.getenv("OPTIROUTE_SERIAL_PORT", "/dev/ttyUSB0")
BAUD_RATE = int(os.getenv("OPTIROUTE_BAUD", "9600"))
HARDWARE_ENABLED = os.getenv("OPTIROUTE_HARDWARE", "0") == "1"

# Calibrate these values for YOUR car. They are starting estimates, not guaranteed distances.
CELL_MS = int(os.getenv("OPTIROUTE_CELL_MS", "650"))
TURN_90_MS = int(os.getenv("OPTIROUTE_TURN_90_MS", "360"))
DEFAULT_SPEED = int(os.getenv("OPTIROUTE_SPEED", "150"))

app = FastAPI(title="OPTIROUTE Robot Bridge", version="1.0.0")
app.mount("/web", StaticFiles(directory=str(WEB_DIR)), name="web")

serial_conn = None
serial_lock = threading.Lock()
state_lock = threading.Lock()
stop_event = threading.Event()

state = {
    "connected": False,
    "hardware_enabled": HARDWARE_ENABLED,
    "mode": "hardware" if HARDWARE_ENABLED else "simulation",
    "running": False,
    "status": "Starting",
    "last_command": "stop",
    "last_message": "",
    "completed_steps": 0,
    "total_steps": 0,
    "error": None,
}

class ExecuteRequest(BaseModel):
    path: List[List[int]]
    rows: int = Field(ge=2, le=30)
    cols: int = Field(ge=2, le=30)
    obstacles: List[List[int]] = []
    speed: int = Field(default=DEFAULT_SPEED, ge=0, le=255)
    heading: str = "N"

def update_state(**kwargs):
    with state_lock:
        state.update(kwargs)

def send_serial(command: str):
    """Send one newline-terminated command using the existing Arduino protocol."""
    update_state(last_command=command)
    if not HARDWARE_ENABLED:
        update_state(last_message=f"SIMULATED: {command}")
        print(f"[SIMULATION] {command}")
        return

    if serial_conn is None or not serial_conn.is_open:
        raise RuntimeError("Arduino serial connection is not available")

    with serial_lock:
        serial_conn.write((command.strip().lower() + "\n").encode("utf-8"))
        serial_conn.flush()
        time.sleep(0.03)
        response = ""
        while serial_conn.in_waiting:
            response += serial_conn.readline().decode(errors="ignore").strip() + " "
        if response.strip():
            update_state(last_message=response.strip())

def stop_motors():
    try:
        send_serial("stop")
    except Exception as exc:
        update_state(error=str(exc), status="Stop command failed")

def motion_for(command: str, duration_ms: int):
    """Keep refreshing the motor command so the Arduino watchdog remains alive."""
    end_time = time.monotonic() + max(0, duration_ms) / 1000.0
    while time.monotonic() < end_time:
        if stop_event.is_set():
            break
        send_serial(command)
        remaining = end_time - time.monotonic()
        stop_event.wait(min(0.22, max(0, remaining)))
    stop_motors()

HEADINGS = ["N", "E", "S", "W"]
VECTORS = {"N": (-1, 0), "E": (0, 1), "S": (1, 0), "W": (0, -1)}

def turn_to(current: str, target: str):
    a, b = HEADINGS.index(current), HEADINGS.index(target)
    diff = (b - a) % 4
    if diff == 0:
        return target
    if diff == 1:
        motion_for("right", TURN_90_MS)
    elif diff == 3:
        motion_for("left", TURN_90_MS)
    else:
        motion_for("right", TURN_90_MS)
        if not stop_event.is_set():
            motion_for("right", TURN_90_MS)
    return target

def validate_path(req: ExecuteRequest):
    if len(req.path) < 2:
        raise ValueError("The route must contain at least a start and destination")
    if req.heading not in HEADINGS:
        raise ValueError("Heading must be N, E, S or W")
    blocked = {tuple(p) for p in req.obstacles}
    for node in req.path:
        if len(node) != 2:
            raise ValueError("Each path node must be [row, column]")
        r, c = node
        if not (0 <= r < req.rows and 0 <= c < req.cols):
            raise ValueError("Path contains a node outside the grid")
        if (r, c) in blocked:
            raise ValueError("Path crosses an obstacle")
    for a, b in zip(req.path, req.path[1:]):
        if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
            raise ValueError("Path contains non-adjacent nodes")
    return True

def execute_worker(req: ExecuteRequest):
    update_state(running=True, status="Executing route", error=None,
                 completed_steps=0, total_steps=len(req.path)-1)
    stop_event.clear()
    current_heading = req.heading
    try:
        send_serial(f"speed {req.speed}")
        for index, (a, b) in enumerate(zip(req.path, req.path[1:]), start=1):
            if stop_event.is_set():
                break
            dr, dc = b[0] - a[0], b[1] - a[1]
            target_heading = next(h for h, v in VECTORS.items() if v == (dr, dc))
            current_heading = turn_to(current_heading, target_heading)
            if stop_event.is_set():
                break
            motion_for("forward", CELL_MS)
            update_state(completed_steps=index)
        stop_motors()
        if stop_event.is_set():
            update_state(status="Stopped by user", last_message="Route stopped")
        else:
            update_state(status="Route completed", last_message="Destination reached")
    except Exception as exc:
        stop_motors()
        update_state(status="Execution error", error=str(exc))
    finally:
        update_state(running=False)

@app.on_event("startup")
def startup():
    if HARDWARE_ENABLED:
        if serial is None:
            update_state(status="Serial library missing", error="Install pyserial")
            return
        try:
            global serial_conn
            serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, write_timeout=1)
            time.sleep(2)  # Arduino Nano may reset when serial opens
            update_state(connected=True, status="Arduino connected", error=None)
            send_serial("stop")
        except Exception as exc:
            update_state(connected=False, status="Arduino connection failed", error=str(exc))
    else:
        update_state(connected=False, status="Simulation mode: hardware disabled",
                     last_message="No motor commands will be sent to the Arduino")

@app.on_event("shutdown")
def shutdown():
    stop_event.set()
    stop_motors()
    if serial_conn and serial_conn.is_open:
        serial_conn.close()

@app.get("/")
def home():
    return FileResponse(WEB_DIR / "index.html")

@app.get("/api/health")
def health():
    return {"ok": True, "service": "OPTIROUTE"}

@app.get("/api/status")
def get_status():
    with state_lock:
        return dict(state, cell_ms=CELL_MS, turn_90_ms=TURN_90_MS,
                    serial_port=SERIAL_PORT, baud_rate=BAUD_RATE)

@app.post("/api/execute")
def execute(req: ExecuteRequest):
    try:
        validate_path(req)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc))
    with state_lock:
        if state["running"]:
            raise HTTPException(status_code=409, detail="A route is already running")
        if HARDWARE_ENABLED and (serial_conn is None or not serial_conn.is_open):
            raise HTTPException(status_code=503, detail="Arduino is not connected")
    stop_event.clear()
    thread = threading.Thread(target=execute_worker, args=(req,), daemon=True)
    thread.start()
    return {"accepted": True, "mode": state["mode"], "message": "Route execution started"}

@app.post("/api/stop")
def stop():
    stop_event.set()
    stop_motors()
    update_state(status="Emergency stop requested")
    return {"ok": True, "message": "Stop requested"}

@app.post("/api/manual/{command}")
def manual(command: str):
    allowed = {"forward", "backward", "left", "right", "stop"}
    if command not in allowed:
        raise HTTPException(status_code=400, detail="Unsupported command")
    if state["running"] and command != "stop":
        raise HTTPException(status_code=409, detail="Stop the route before manual control")
    try:
        if command == "stop":
            stop_event.set()
            stop_motors()
            update_state(status="Stopped by user")
        else:
            send_serial(command)
        return {"ok": True, "command": command, "mode": state["mode"]}
    except Exception as exc:
        raise HTTPException(status_code=503, detail=str(exc))

@app.post("/api/speed/{value}")
def set_speed(value: int):
    if not 0 <= value <= 255:
        raise HTTPException(status_code=400, detail="Speed must be 0–255")
    try:
        send_serial(f"speed {value}")
        return {"ok": True, "speed": value}
    except Exception as exc:
        raise HTTPException(status_code=503, detail=str(exc))
