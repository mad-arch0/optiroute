#!/bin/bash
set -e
cd "$(dirname "$0")"
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
echo "Starting OPTIROUTE in SAFE SIMULATION MODE."
echo "Open http://<RASPBERRY_PI_IP>:8000 from a device on the same Wi-Fi."
uvicorn pi_server:app --host 0.0.0.0 --port 8000
