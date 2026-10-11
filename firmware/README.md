# Firmware (flight computer)

Code that runs on the microcontroller. The target is a **Teensy 4.0**; early prototyping used an Arduino Uno / SparkFun RedBoard.

| Folder | Board | What it is |
|---|---|---|
| `flight_computer/` | Teensy 4.0 | **Main controller.** IMU → PID → HTD-30H bus servo, EDF throttle, safety states. Start here. |
| `tests/servo_bus_test/` | Teensy 4.0 | First power-up of the servo through the BusLinker |
| `tests/esc_test/` | Teensy 4.0 | ESC throttle calibration and EDF current steps |
| `tests/hardware_test/` | Uno | IMU readout + PWM servo sweep (old hardware) |
| `tests/RotationMappedToServoTest/` | Uno | IMU angle mapped to a PWM servo (old hardware) |
| `tvc_controller/` | Uno | Earlier closed-loop controller for a **PWM** servo. Kept for reference; it can't drive the HTD-30H. |

Arduino requires every sketch to be in a folder with the same name as its `.ino` file. To add a test, create `tests/my_test/my_test.ino`.

Compile from the command line (optional):
```bash
arduino-cli compile --fqbn teensy:avr:teensy40 firmware/flight_computer
```
