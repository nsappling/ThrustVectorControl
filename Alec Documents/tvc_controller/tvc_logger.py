#!/usr/bin/env python3
"""tvc_logger.py -- run on the Raspberry Pi (or any laptop) with the RedBoard
plugged in over USB.

Saves the RedBoard's CSV stream to a timestamped file (open it in MATLAB with
readtable) and lets you type tuning commands (kp 0.5, kd 0.035, zero,
center, run, help) that get sent straight to the RedBoard.

    pip install pyserial
    python3 tvc_logger.py                 # auto-picks the port
    python3 tvc_logger.py /dev/ttyUSB0    # or name it explicitly

Ctrl+C to quit.
"""
import sys
import threading
import time
from datetime import datetime

import serial
from serial.tools import list_ports

BAUD = 115200
HEADER = "t_ms,theta_deg,theta_filt_deg,u,fan_tilt_deg,servo_us"


def find_port():
    for p in list_ports.comports():
        # RedBoard / clones show up as CH340 (ttyUSB*), genuine Arduinos as ACM
        if any(s in (p.device + str(p.description)) for s in ("USB", "ACM", "usbserial", "CH340")):
            return p.device
    sys.exit("No Arduino found -- pass the port name, e.g. /dev/ttyUSB0")


def reader(ser, f):
    last_print = 0.0
    while True:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        if line.startswith("t_ms"):
            continue                         # board's own CSV header (already written above)
        if line.startswith("#"):
            print(line)                      # status messages from the board
            continue
        f.write(line + "\n")
        now = time.time()
        if now - last_print > 0.5:           # don't flood the terminal
            parts = line.split(",")
            if len(parts) == 6:
                print(f"  theta={parts[2]:>7} deg   fan={parts[4]:>6} deg   servo={parts[5]} us")
            last_print = now


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    fname = datetime.now().strftime("tvc_log_%Y%m%d_%H%M%S.csv")
    ser = serial.Serial(port, BAUD, timeout=1)
    print(f"Connected to {port}. Logging to {fname}. Type commands, Ctrl+C to quit.")
    with open(fname, "w", buffering=1) as f:
        f.write(HEADER + "\n")
        threading.Thread(target=reader, args=(ser, f), daemon=True).start()
        try:
            while True:
                cmd = input()
                ser.write((cmd.strip() + "\n").encode())
        except (KeyboardInterrupt, EOFError):
            print(f"\nSaved {fname}")


if __name__ == "__main__":
    main()
