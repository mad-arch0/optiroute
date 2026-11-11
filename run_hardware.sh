#!/bin/bash
set -e
cd "$(dirname "$0")"
source .venv/bin/activate
export OPTIROUTE_HARDWARE=1
# Change this if Arduino appears as /dev/ttyACM0 instead.
export OPTIROUTE_SERIAL_PORT=/dev/ttyUSB0
export OPTIROUTE_BAUD=9600
# Calibrate these values with the car raised off the floor first.
export OPTIROUTE_CELL_MS=650
export OPTIROUTE_TURN_90_MS=360
export OPTIROUTE_SPEED=120
uvicorn pi_server:app --host 0.0.0.0 --port 8000
