#!/usr/bin/env python3
"""tvc_logger.py -- run on the Raspberry Pi (or any laptop) with the flight
computer (Teensy 4.0, or the older Uno/RedBoard) plugged in over USB.

Saves the board's CSV stream to a timestamped file (open it in MATLAB with
readtable) and lets you type commands (help, arm, stop, kp 5, ...) that get
sent straight to the board. Works with any sketch that follows the format in
docs/icd/usb_link.md: '#' lines are status, the 't_ms,...' line is the CSV
header, everything else is data.

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

BAUD = 115200   # ignored by the Teensy (native USB), needed by the Uno


def find_port():
    for p in list_ports.comports():
        # Teensy and genuine Arduinos show up as ttyACM*, RedBoard/CH340 clones as ttyUSB*
        if any(s in (p.device + str(p.description)) for s in ("Teensy", "ACM", "USB", "usbserial", "CH340")):
            return p.device
    sys.exit("No board found -- pass the port name, e.g. /dev/ttyUSB0")


def reader(ser, f):
    last_print = 0.0
    header = []
    while True:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        if line.startswith("#"):
            print(line)                      # status messages from the board
            continue
        if line.startswith("t_ms"):
            if not header:                   # board's CSV header: write it once
                f.write(line + "\n")
            header = line.split(",")
            continue
        f.write(line + "\n")
        now = time.time()
        if now - last_print > 0.5 and header:   # don't flood the terminal
            parts = line.split(",")
            if len(parts) == len(header):
                print("  " + "  ".join(f"{h}={v}" for h, v in zip(header[1:], parts[1:])))
            last_print = now


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    fname = datetime.now().strftime("tvc_log_%Y%m%d_%H%M%S.csv")
    ser = serial.Serial(port, BAUD, timeout=1)
    print(f"Connected to {port}. Logging to {fname}. Type commands, Ctrl+C to quit.")
    with open(fname, "w", buffering=1) as f:
        threading.Thread(target=reader, args=(ser, f), daemon=True).start()
        try:
            while True:
                cmd = input()
                ser.write((cmd.strip() + "\n").encode())
        except (KeyboardInterrupt, EOFError):
            print(f"\nSaved {fname}")


if __name__ == "__main__":
    main()
