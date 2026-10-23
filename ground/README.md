# Ground computer (Raspberry Pi 5)

Code that runs on the Raspberry Pi 5 (or any Linux/Mac laptop) connected to the Teensy 4.0 over USB. The Pi **supervises**: it flashes firmware, logs data, and sends commands. The Teensy runs the actual control loop, so control never depends on the Pi's timing.

| File | What it does |
|---|---|
| `setup_pi.sh` | **One-time setup.** Installs pyserial, the Teensy loader, Teensy udev rules, serial-port permission, and arduino-cli with Teensy support. |
| `flash.sh` | Compiles any sketch folder and uploads it to the Teensy: `bash ground/flash.sh firmware/tests/imu_test` |
| `tvc_logger.py` | Console + logger. Prints the Teensy's status, saves CSV to `ground/logs/`, sends what you type. With `flight_computer` it also sends the safety heartbeat and sends `stop` when you quit. |

## Quick start (on the Pi, from the repo folder)
```bash
bash ground/setup_pi.sh
```
Log out and back in once, then:
```bash
bash ground/flash.sh firmware/tests/usb_hello
```
```bash
python3 ground/tvc_logger.py
```

The full step-by-step bring-up (which sketch, which wiring, what "pass" looks like) is in **[docs/bringup/README.md](../docs/bringup/README.md)**.

Close `tvc_logger.py` before running `flash.sh`: only one program can use the Teensy's port at a time.
