# CLAUDE.md

Senior design project: an interactive thrust vector control (TVC) demonstrator. An electric ducted fan (EDF) on a single-axis servo gimbal stabilizes a rocket body mounted on a pivot below its CG (an inverted pendulum). See README.md for the full concept.

## Repo layout

- `Alec Documents/rocketgimball.m` — MATLAB simulation of the TVC plant + PID autopilot with live animation and keyboard manual mode (port of an earlier `rocketgimball.py`).
- `*.SLDPRT` / `*.SLDASM` — SolidWorks CAD (binary; do not edit). Top-level assembly: `Rocket_Assembly.SLDASM`.
- `CAD_Equations.txt` — SolidWorks global variables (body tube, bearing, pivot dimensions, in inches).
- `Alec Documents/`, `Kailash Documents/` — per-member work folders; `Kailash Documents/COTS PARTS/` holds vendor parts (e.g. DS3218MG servo, McMaster bearings/screws).

## MATLAB

- Version: R2026a (R2024b and R2025b are also installed). Toolboxes: Control System Toolbox, Signal Processing Toolbox, Simulink, Simulink Control Design.
- Run headless: `matlab -batch "..."` (about 10 s startup per call). `matlab` is on PATH via `~/.zprofile`; if not found, use `/Applications/MATLAB_R2026a.app/bin/matlab`.
- Lint: `matlab -batch "checkcode('Alec Documents/rocketgimball.m')"`.
- `rocketgimball.m` runs an infinite `while ishandle(fig)` animation loop, so **don't run it with `-batch`** (it hangs). To test the physics or control logic headlessly, pull the dynamics into a function or a separate script with a fixed step count and no graphics.

## Model notes (rocketgimball.m)

- Plant: `theta_ddot = b*u + a*sin(theta)`, with `a = g/r_0` and `b = F_t*R/(m*r_0^2)`. Physical parameters (`F_t`, `R`, `m`, `r_0`) are still guesses — flag this when results depend on them.
- `b` must stay positive (negative feedback). Actuator saturation `Umax_actuator = fan_tilt_max_deg / VIS_SCALE` applies to the physics in both AUTO and MANUAL modes.
- The drawn fan tilt is `-u * VIS_SCALE` (sign flipped because of the clockwise-positive `rotate_pt` convention). `VIS_SCALE` is cosmetic, but it also sets `Umax_actuator`, so changing it changes the physics.
- Integration: semi-implicit Euler, `dt = 0.0005` s, 4 substeps per frame. Mechanical stops are modeled as inelastic stops at ±`theta_limit`.

## Conventions

- Keep the heavy explanatory commenting style in existing MATLAB files.
- Team repo shared with Nate, Warren, and Kailash: don't commit or push unless asked, and don't rewrite others' folders without being asked.
