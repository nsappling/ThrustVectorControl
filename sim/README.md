# Simulation (MATLAB)

| Folder | What it is | Start with |
|---|---|---|
| `tvc_sim/` | Full design sim: nonlinear plant, servo/EDF/IMU models, trade studies, servo sizing. **Use this for design decisions.** | `tvc_main` |
| `pid_animation/` | Simpler PID sim with a live animation and a MANUAL mode flown with an on-screen dial, plus `analyze_behavior` (time constants, overshoot, settling, margins, disturbance rejection; figures in `results/behavior/`), a headless gain sweep, and a serial logger. `tvc_controller.ino` mirrors its PID, so keep gains and sign conventions in sync. | `rocketgimball` |

`pid_animation/rocketgimball.m` runs an infinite animation loop, so don't run it with `matlab -batch`. Use `tvc_sim.m`, `tune_pid.m`, or `analyze_behavior.m` for headless runs.
