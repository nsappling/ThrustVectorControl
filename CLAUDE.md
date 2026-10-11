# CLAUDE.md

Senior design project: 1-DOF thrust vector control demonstrator. An EDF on a single-axis servo gimbal stabilizes a rocket body on a pivot below its CG (an inverted pendulum). The end goal is an interactive science-center exhibit with a joystick. See `README.md`.

Team: Nate Sapp, Warren Schindler, Kailash Gupta-Verma, Alec Garcia.

## Layout

- `firmware/`: flight computer. Arduino Uno / SparkFun RedBoard now; the target is a **Teensy 4.0** (3.3 V logic, unlike the 5 V Uno, so watch logic levels).
  - `tvc_controller/`: main loop. GY-521 (MPU-6050) on I2C, servo signal on D4, PID at 500 Hz, CSV telemetry at 115200 baud. Its README has wiring and the bench-test checklist.
  - `tests/`: single-purpose hardware sketches. Every sketch must be in a folder with the same name as its `.ino` file.
- `ground/`: Raspberry Pi 5 code. `tvc_logger.py` logs the CSV and sends tuning commands over USB serial.
- `sim/tvc_sim/`: main MATLAB design sim (`tvc_main`). Every parameter is in `tvc_params.m`, tagged `[EST]` / `[SPEC]` / `[DES]`.
- `sim/pid_animation/`: simple PID sim (`rocketgimball.m`, animated; **never run it with `-batch`**, it loops forever), a headless copy (`tvc_sim.m`), a gain sweep (`tune_pid.m`), and a serial logger (`log_serial.m`). `tvc_controller.ino` mirrors this PID, so keep gains and sign conventions in sync.
- `cad/`: SolidWorks (binary, never edit). Each assembly is in the same folder as its parts; move them together.
- `docs/`: sketches; ICDs coming.

## Hardware (planned, still being refined)

- 12 V single-output PSU (600 W).
- EDF branch: 15 A fuse → 80 A ESC → EDF.
- Servo branch: 5 A fuse → step-down/servo supply → servo.
- Raspberry Pi 5 ground computer on its own 5 V wall supply, connected to the flight computer over USB.
- Never power the servo from the MCU's 5 V pin. The servo supply ground and the MCU ground must be common.

## Workflow

- The team prototypes in the Arduino IDE and asks Claude to edit sketches. Keep sketches compiling and self-contained, and keep the heavy explanatory commenting style used in the existing files.
- Compile check (if arduino-cli is available): `arduino-cli compile --fqbn arduino:avr:uno firmware/tvc_controller`. For Teensy: `--fqbn teensy:avr:teensy40`.
- Physical parameters (thrust, masses, lever arms) are mostly estimates. Flag this when a result depends on them.
- Use clear commit messages that say what changed and whether it was tested on hardware.
