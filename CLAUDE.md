# CLAUDE.md

Senior design project: an interactive thrust vector control (TVC) demonstrator. An electric ducted fan (EDF) on a single-axis servo gimbal stabilizes a rocket body mounted on a pivot below its CG (an inverted pendulum). See README.md for the full concept.

## Repo layout

- `Alec Documents/rocketgimball.m` — MATLAB simulation of the TVC plant + PID autopilot with live animation and keyboard manual mode (port of an earlier `rocketgimball.py`).
- `Alec Documents/tvc_controller/`: Arduino firmware for a SparkFun RedBoard, Uno-compatible (`tvc_controller.ino`), a Pi/laptop serial logger (`tvc_logger.py`), and wiring plus bench-test steps (`README_hardware.md`). The firmware mirrors the PID in `rocketgimball.m`, so keep gains and sign conventions in sync. Hardware: GY-521 (MPU-6050), MIUZEI 20 kg 180° servo on D4 (direct drive to the gimbal), IMU on SDA/SCL, RedBoard, Pi 5. Compile check: `"/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli" compile --fqbn arduino:avr:uno "Alec Documents/tvc_controller"`.
- `Alec Documents/hardware_test/hardware_test.ino`: a simple servo sweep that prints IMU readings as CSV, for checking wiring.
- `Alec Documents/log_serial.m`: records the Arduino's CSV serial output (from either sketch) to `Alec Documents/data/log_*.csv`, with a live plot. Close the Arduino Serial Monitor before running it, since only one program can use the port.
- `*.SLDPRT` / `*.SLDASM` — SolidWorks CAD (binary; do not edit). Top-level assembly: `Rocket_Assembly.SLDASM`.
- `CAD_Equations.txt` — SolidWorks global variables (body tube, bearing, pivot dimensions, in inches).
- `Alec Documents/`, `Kailash Documents/` — per-member work folders; `Kailash Documents/COTS PARTS/` holds vendor parts (e.g. DS3218MG servo, McMaster bearings/screws).

## MATLAB

- Version: R2026a (R2024b and R2025b are also installed). Toolboxes: Control System Toolbox, Signal Processing Toolbox, Simulink, Simulink Control Design.
- Run headless: `matlab -batch "..."` (about 10 s startup per call). `matlab` is on PATH via `~/.zprofile`; if not found, use `/Applications/MATLAB_R2026a.app/bin/matlab`.
- Lint: `matlab -batch "checkcode('Alec Documents/rocketgimball.m')"`.
- `rocketgimball.m` runs an infinite `while ishandle(fig)` animation loop, so **don't run it with `-batch`** (it hangs). Use the headless tools instead:
  - `Alec Documents/tvc_sim.m`: the same AUTO-mode physics and PID with no graphics, e.g. `out = tvc_sim(struct('Kp',0.5), 3)`. Its defaults mirror `rocketgimball.m`, so keep the two in sync.
  - `Alec Documents/tune_pid.m`: baseline metrics, a linear stability check, and a Kp x Kd sweep. Plots go to `Alec Documents/results/`. Run it with `matlab -batch "run('Alec Documents/tune_pid.m')"` (takes about 30 s).

## Model notes (rocketgimball.m)

- Plant: `theta_ddot = b*u + a*sin(theta)`, with `a = g/r_0` and `b = F_t*R/(m*r_0^2)`. Physical parameters (`F_t`, `R`, `m`, `r_0`) are still guesses — flag this when results depend on them.
- `b` must stay positive (negative feedback). Actuator saturation `Umax_actuator = fan_tilt_max_deg / VIS_SCALE` applies to the physics in both AUTO and MANUAL modes.
- The drawn fan tilt is `-u * VIS_SCALE` (sign flipped because of the clockwise-positive `rotate_pt` convention). `VIS_SCALE` is cosmetic, but it also sets `Umax_actuator`, so changing it changes the physics.
- Integration: semi-implicit Euler, `dt = 0.0005` s, 4 substeps per frame. Mechanical stops are modeled as inelastic stops at ±`theta_limit`.

## Conventions

- Keep the heavy explanatory commenting style in existing MATLAB files.
- Team repo shared with Nate, Warren, and Kailash: don't commit or push unless asked, and don't rewrite others' folders without being asked.
