#!/usr/bin/env python3
"""tvc_logger.py -- the ground-station console. Runs on the Raspberry Pi (or
any laptop) with the Teensy (or the older Uno/RedBoard) plugged in over USB.

  * Prints the board's status lines ('#...') and a summary of its data.
  * Saves every CSV data row to ground/logs/tvc_log_YYYYMMDD_HHMMSS.csv
    (open it in MATLAB with readtable).
  * Sends whatever you type (help, arm, stop, kp 5, ...) to the board.
  * When it detects flight_computer (its CSV header has a 'state' column):
      - sends a heartbeat ('hb') 4x per second, so flight_computer can drop
        to SAFE if the Pi or this program dies (set HOST_TIMEOUT_MS in
        config.h to use it), and
      - sends 'stop' when you quit (Ctrl+C), so the EDF never keeps
        running after the console is closed.

Works with any sketch that follows docs/icd/usb_link.md: '#' lines are
status, the 't_ms,...' line is the CSV header, everything else is data.

    python3 ground/tvc_logger.py                 # auto-picks the Teensy
    python3 ground/tvc_logger.py /dev/ttyACM0    # or name the port
    python3 ground/tvc_logger.py --no-hb         # never send heartbeats

Needs pyserial (ground/setup_pi.sh installs it; or: pip install pyserial).
"""
import argparse
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

BAUD = 115200          # ignored by the Teensy (native USB); needed by the Uno
TEENSY_VID = 0x16C0    # PJRC's USB vendor ID
HB_PERIOD_S = 0.25     # heartbeat interval (flight_computer only)
LOG_DIR = Path(__file__).resolve().parent / "logs"


def find_port():
    """Prefer a Teensy (by USB vendor ID); fall back to anything Arduino-like."""
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid == TEENSY_VID:
            return p.device
    for p in ports:
        # Genuine Arduinos show up as ttyACM*, RedBoard/CH340 clones as ttyUSB*
        if any(s in (p.device + str(p.description)) for s in ("ACM", "ttyUSB", "usbserial", "CH340")):
            return p.device
    sys.exit("No board found. Is the USB cable a DATA cable? Is another program "
             "(Arduino Serial Monitor, flash.sh) using the port? Or pass the port name, "
             "e.g. /dev/ttyACM0")


class Console:
    def __init__(self, ser, log_file, allow_hb):
        self.ser = ser
        self.log = log_file
        self.allow_hb = allow_hb
        self.header = []
        self.is_flight_computer = False   # set once we see its CSV header
        self.rows = 0
        self.pending = []                 # data rows that arrived before the header
        self.asked_header = 0.0           # when we sent 'header' (0 = not yet)
        self.lock = threading.Lock()      # one writer at a time on the serial port
        self.running = True

    def send(self, text):
        with self.lock:
            self.ser.write((text.strip() + "\n").encode())

    # ----------------------------------------------------------------- reader
    def reader(self):
        last_print = 0.0
        while self.running:
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                print("\n!! Lost the USB connection (Teensy unplugged or rebooted). Quit and restart.")
                self.running = False
                return
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                print(line)                                   # status from the board
                continue
            if line.startswith("t_ms"):                       # CSV header (sent at boot)
                if not self.header:
                    self.log.write(line + "\n")
                    for row in self.pending:                  # rows held while we waited
                        self.log.write(row + "\n")
                    self.rows += len(self.pending)
                    self.pending = []
                self.header = line.split(",")
                if "state" in self.header and not self.is_flight_computer:
                    self.is_flight_computer = True
                    if self.allow_hb:
                        print("   [flight_computer detected: sending heartbeat; will send 'stop' on exit]")
                continue
            if not self.header:
                # Connected after the board booted, so we missed its one-time
                # header. Ask for it (sketches following docs/icd/usb_link.md
                # answer 'header') and hold rows so the file starts with it.
                if not self.asked_header:
                    self.asked_header = time.time()
                    print("   [connected mid-stream: asking the board to re-send its CSV header]")
                    self.send("header")
                if time.time() - self.asked_header < 2.0:
                    self.pending.append(line)
                    continue
                if self.pending:                              # board didn't answer: save rows anyway
                    print("   [no header from the board -- log file will have no column names]")
                    for row in self.pending:
                        self.log.write(row + "\n")
                    self.rows += len(self.pending)
                    self.pending = []
            self.log.write(line + "\n")
            self.rows += 1
            now = time.time()
            if now - last_print > 0.5 and self.header:        # don't flood the terminal
                parts = line.split(",")
                if len(parts) == len(self.header):
                    print("  " + "  ".join(f"{h}={v}" for h, v in zip(self.header[1:], parts[1:])))
                last_print = now

    # -------------------------------------------------------------- heartbeat
    def heartbeat(self):
        while self.running:
            if self.allow_hb and self.is_flight_computer:
                try:
                    self.send("hb")
                except serial.SerialException:
                    return
            time.sleep(HB_PERIOD_S)


def main():
    ap = argparse.ArgumentParser(description="TVC ground-station console and logger")
    ap.add_argument("port", nargs="?", help="serial port (default: auto-detect the Teensy)")
    ap.add_argument("--no-hb", action="store_true", help="never send heartbeats")
    args = ap.parse_args()

    port = args.port or find_port()
    LOG_DIR.mkdir(exist_ok=True)
    fname = LOG_DIR / datetime.now().strftime("tvc_log_%Y%m%d_%H%M%S.csv")
    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Can't open {port}: {e}\n(Another program using it? Not in the dialout group yet? Log out/in after setup_pi.sh.)")

    print(f"Connected to {port}. Logging to {fname}")
    print("Type commands and press Enter ('help' lists them). Ctrl+C to quit.\n")
    with open(fname, "w", buffering=1) as f:
        con = Console(ser, f, allow_hb=not args.no_hb)
        threading.Thread(target=con.reader, daemon=True).start()
        threading.Thread(target=con.heartbeat, daemon=True).start()
        try:
            while con.running:
                cmd = input()
                if cmd.strip():
                    con.send(cmd)
        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            if con.is_flight_computer:
                try:
                    con.send("stop")                          # never leave the EDF running
                    time.sleep(0.1)
                    print("\nSent 'stop' to flight_computer.")
                except serial.SerialException:
                    pass
            con.running = False
            print(f"Saved {con.rows} rows to {fname}")


if __name__ == "__main__":
    main()
