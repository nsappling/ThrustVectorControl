#!/usr/bin/env bash
# setup_pi.sh -- one-time setup of a Raspberry Pi 5 as the TVC ground computer.
#
# Run once, from the repo folder on the Pi:
#     bash ground/setup_pi.sh
# then LOG OUT AND BACK IN (or reboot) so the serial-port permission applies.
#
# Needs: Raspberry Pi OS 64-bit (Bookworm or newer) and internet access.
# Safe to re-run: every step checks whether it's already done.
#
# What it installs and why:
#   1. python3-serial      -- tvc_logger.py talks to the Teensy over USB serial
#   2. teensy-loader-cli   -- flash.sh uses it to upload firmware to the Teensy
#   3. dialout group       -- lets your user open /dev/ttyACM0 without sudo
#   4. PJRC udev rules     -- lets non-root users see/flash the Teensy (PJRC requires these on Linux)
#   5. arduino-cli + Teensy core -- compiles the .ino sketches right on the Pi

set -euo pipefail

say() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }

say "1/5  System packages (python3-serial, teensy-loader-cli, curl, git)"
sudo apt-get update
sudo apt-get install -y python3-serial teensy-loader-cli curl git

say "2/5  Serial port permission (dialout group)"
if id -nG "$USER" | grep -qw dialout; then
    echo "already in dialout"
else
    sudo usermod -aG dialout "$USER"
    echo "added $USER to dialout -- LOG OUT AND BACK IN before using the Teensy"
fi

say "3/5  Teensy udev rules (from pjrc.com)"
if [ -f /etc/udev/rules.d/00-teensy.rules ]; then
    echo "already installed"
else
    curl -fsSL https://www.pjrc.com/teensy/00-teensy.rules | sudo tee /etc/udev/rules.d/00-teensy.rules > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "installed -- unplug and re-plug the Teensy if it was connected"
fi

say "4/5  arduino-cli (into ~/.local/bin)"
mkdir -p "$HOME/.local/bin"
export PATH="$HOME/.local/bin:$PATH"
if command -v arduino-cli > /dev/null; then
    echo "already installed: $(arduino-cli version)"
else
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$HOME/.local/bin" sh
fi
if ! grep -q '.local/bin' "$HOME/.bashrc" 2> /dev/null; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
fi

say "5/5  Teensy board support for arduino-cli (~200 MB download, a few minutes)"
arduino-cli config init > /dev/null 2>&1 || true   # creates a config file only if none exists
arduino-cli config add board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json 2> /dev/null || true
arduino-cli core update-index
arduino-cli core install teensy:avr

say "Done. Quick check:"
echo "  arduino-cli:       $(command -v arduino-cli)"
echo "  teensy_loader_cli: $(command -v teensy_loader_cli)"
echo "  Teensy core:       $(arduino-cli core list | grep -i teensy || echo 'MISSING')"
echo
echo "Next: log out and back in, then follow docs/bringup/README.md, stage 1:"
echo "  bash ground/flash.sh firmware/tests/usb_hello"
