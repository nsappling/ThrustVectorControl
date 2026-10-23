# Firmware (flight computer)

Code that runs on the microcontroller. The target is a **Teensy 4.0**; early prototyping used an Arduino Uno / SparkFun RedBoard.

| Folder | Board | What it is |
|---|---|---|
| `flight_computer/` | Teensy 4.0 | **Main controller.** IMU → PID → HTD-30H bus servo, EDF throttle, safety states. Start here. |
| `tests/usb_hello/` | Teensy 4.0 | Bring-up stage 1: blink + USB serial echo (proves flashing from the Pi works) |
| `tests/imu_test/` | Teensy 4.0 | Bring-up stage 2: I2C scan, WHO_AM_I, raw IMU data, noise check, axis/sign check |
| `tests/uart_loopback_test/` | Teensy 4.0 | Bring-up stage 3: Serial1 through the level shifter with an HV2→HV1 jumper |
| `tests/servo_bus_test/` | Teensy 4.0 | First power-up of the servo through the BusLinker |
| `tests/esc_test/` | Teensy 4.0 | ESC throttle calibration and EDF current steps |
| `tests/hardware_test/` | Uno | IMU readout + PWM servo sweep (old hardware) |
| `tests/RotationMappedToServoTest/` | Uno | IMU angle mapped to a PWM servo (old hardware) |
| `tvc_controller/` | Uno | Earlier closed-loop controller for a **PWM** servo. Kept for reference; it can't drive the HTD-30H. |

Arduino requires every sketch to be in a folder with the same name as its `.ino` file. To add a test, create `tests/my_test/my_test.ino`.

**Bring-up order** (which sketch, which wiring, what "pass" looks like): [docs/bringup/README.md](../docs/bringup/README.md). From the Pi, flash any sketch with `bash ground/flash.sh <folder>`.

Compile from the command line (optional):
```bash
arduino-cli compile --fqbn teensy:avr:teensy40 firmware/flight_computer
```
