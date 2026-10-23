#!/usr/bin/env bash
# flash.sh -- compile an Arduino sketch and upload it to the Teensy 4.0,
# all from the Raspberry Pi. Run from the repo folder:
#
#     bash ground/flash.sh firmware/tests/usb_hello
#     bash ground/flash.sh firmware/flight_computer
#
# Close tvc_logger.py (or anything else using /dev/ttyACM0) first.
# One-time setup: bash ground/setup_pi.sh
#
# How it works:
#   1. arduino-cli compiles the sketch folder into a .hex file
#      (board: Teensy 4.0, USB Type: Serial).
#   2. teensy_loader_cli asks the running Teensy to reboot into its
#      bootloader (-s), waits for it (-w), and writes the .hex.
#      If the Teensy is running something that doesn't answer the reboot
#      request, it just waits: PRESS THE WHITE BUTTON on the Teensy.

set -euo pipefail
export PATH="$HOME/.local/bin:$PATH"

SKETCH="${1:-}"
if [ -z "$SKETCH" ] || [ ! -d "$SKETCH" ]; then
    echo "usage: bash ground/flash.sh <sketch folder>"
    echo "   e.g. bash ground/flash.sh firmware/tests/imu_test"
    exit 1
fi
SKETCH="${SKETCH%/}"
NAME="$(basename "$SKETCH")"
if [ ! -f "$SKETCH/$NAME.ino" ]; then
    echo "error: $SKETCH/$NAME.ino not found (Arduino needs the folder and .ino to share a name)"
    exit 1
fi

FQBN="teensy:avr:teensy40:usb=serial"
BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT

echo "==> Compiling $SKETCH for Teensy 4.0"
arduino-cli compile --fqbn "$FQBN" --output-dir "$BUILD" "$SKETCH"

HEX="$BUILD/$NAME.ino.hex"
if [ ! -f "$HEX" ]; then
    echo "error: compile finished but $HEX is missing"
    exit 1
fi

echo "==> Uploading $NAME"
echo "    (if this sits at 'Waiting for Teensy device...', press the white button on the Teensy)"
teensy_loader_cli --mcu=TEENSY40 -w -s -v "$HEX"

echo "==> Done. The Teensy rebooted into $NAME."
echo "    Talk to it with:  python3 ground/tvc_logger.py"
