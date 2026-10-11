# Firmware (flight computer)

Code that runs on the microcontroller in the rocket. Today that's an Arduino Uno / SparkFun RedBoard; the target is a Teensy 4.0.

| Folder | What it is |
|---|---|
| `tvc_controller/` | Main closed-loop controller (IMU → PID → servo). Read its `README.md` for wiring and the bench-test checklist. |
| `tests/` | Small standalone sketches for checking one piece of hardware at a time. |

Arduino requires every sketch to be in a folder with the same name as its `.ino` file. To add a test, create `tests/my_test/my_test.ino`.
