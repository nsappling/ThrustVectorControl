# CLAUDE.md

Senior design project: 1-DOF thrust vector control demonstrator. An EDF on a single-axis servo gimbal stabilizes a rocket body on a pivot below its CG (an inverted pendulum). The end goal is an interactive science-center exhibit with a joystick. See `README.md`.

Team: Nate Sapp, Warren Schindler, Kailash Gupta-Verma, Alec Garcia.

## Layout

- `docs/`: **read `docs/system_overview.md` first for any hardware question.** `docs/pinout.md` is the single source of truth for Teensy pins. `docs/icd/*.md` has one interface control document per interface (power, edf_esc, servo_bus, imu_i2c, usb_link). `docs/wiring/tvc_harness.yml` is the WireViz source for the harness diagram; regenerate with `wireviz docs/wiring/tvc_harness.yml` after editing it.
- `firmware/flight_computer/`: Teensy 4.0 main controller. `config.h` holds all pins, limits, and gains and must match `docs/pinout.md`. States: SAFE → ARMED (gimbal only) → RUN (EDF on) / FAULT.
- `firmware/tests/`: single-purpose bring-up sketches (`servo_bus_test`, `esc_test` for Teensy; older Uno ones). Every sketch must be in a folder with the same name as its `.ino` file. Test sketches are self-contained on purpose.
- `firmware/tvc_controller/`: older Uno controller for a PWM servo; reference only.
- `ground/`: Raspberry Pi 5 code. `tvc_logger.py` logs the CSV and sends commands over USB serial.
- `sim/tvc_sim/`: main MATLAB design sim (`tvc_main`). Every parameter is in `tvc_params.m`, tagged `[EST]` / `[SPEC]` / `[DES]`. `flight_computer` uses the same control law; keep gains and sign conventions in sync.
- `sim/pid_animation/`: simple animated PID sim (`rocketgimball.m`, a uifigure with an AUTO/MANUAL switch, a fan-tilt dial and a sim-speed dropdown; **never run it with `-batch`**, it loops forever) and related tools. `analyze_behavior.m` is the headless behavior report (figures and `summary.txt` in `results/behavior/`). `tvc_sim.m` defaults must match `rocketgimball.m`. It mirrors `firmware/tvc_controller`.
- `cad/`: SolidWorks (binary, never edit). Each assembly is in the same folder as its parts; move them together.

## Hardware (preliminary, see docs/)

- Mean Well LRS-600-12 PSU (12 V, 50 A). EDF branch: 15 A fuse, 12 AWG, XT60 → Hobbywing Skywalker 50A V2 ESC → JFtech 64 mm 4300 KV EDF (3.5 mm bullets). Servo branch: 5 A fuse, 18 AWG → Hiwonder BusLinker V3 → HTD-30H bus servo (runs directly on 12 V).
- Teensy 4.0: **3.3 V logic, not 5 V tolerant.** Powered from Pi USB. The ESC's BEC is not used. The Teensy ↔ BusLinker UART goes through a BSS138 level shifter.
- The HTD-30H is a **serial bus servo** (Hiwonder protocol, 115200 baud, 0–1000 counts = 0–240°), not PWM.
- The EDF can draw ~50 A, so firmware caps throttle (`THROTTLE_MAX`) to stay under the 15 A fuse.
- Numbers tagged [VERIFY] in the docs are unconfirmed; don't present them as fact.

## Workflow

- The team prototypes in the Arduino IDE and asks Claude to edit sketches. Keep sketches compiling and self-contained, and keep the heavy explanatory commenting style used in the existing files.
- Compile check (if arduino-cli is available): `arduino-cli compile --fqbn teensy:avr:teensy40 firmware/flight_computer`. Uno sketches: `--fqbn arduino:avr:uno`.
- When wiring or pins change, update `docs/pinout.md`, the relevant ICD, `config.h`, and `tvc_harness.yml` in the same commit.
- Physical parameters (thrust, masses, lever arms) are mostly estimates. Flag this when a result depends on them.
- Use clear commit messages that say what changed and whether it was tested on hardware.
