# 1-DOF Gimbaled-EDF TVC Demonstrator: MATLAB design simulation

Plain MATLAB (R2019b or newer). No toolboxes needed.

## Quick start
1. Open MATLAB in this folder.
2. Run `tvc_main`. It prints the linear design report, then runs the three demo tests (controller OFF, controller ON, human pilot), plots θ, θ̇, δ, and prints the metrics.
3. Run sections of `tvc_design_sweeps` (Ctrl+Enter inside a `%%` block) for the trade studies.
4. Run `tvc_servo_sizing` for the gimbal servo torque/speed check.

## Files
| File | What it does |
|---|---|
| `tvc_params.m` | **Every number in one place.** Each one is tagged `[EST]`, `[SPEC]`, or `[DES]`. |
| `tvc_simulate.m` | Nonlinear plant (RK4) plus discrete IMU, filter, controller, and servo PWM frame |
| `tvc_scenario.m` | Test definitions: push size/timing, mode, throttle, reference angle |
| `tvc_metrics.m` | Settling time, overshoot, max deviation, steady-state error, gimbal saturation, jitter |
| `tvc_design_gains.m` | PD pole placement: `Kp = (wn² + mgh/I)/(TL/I)`, `Kd = 2ζwn/(TL/I)` |
| `tvc_linear_report.m` | Unstable pole, authority ratio, minimum throttle, latency budget, delay margin, human-delay limit |
| `tvc_design_sweeps.m` | Eight trade studies (see the header of the file) |
| `tvc_servo_sizing.m` | Inertia → acceleration → torque → ×2 margin → candidate torque-speed lines |
| `tvc_animate.m` | Side-view animation of any run |
| `edf_thrust.m` | Thrust vs throttle: power law, or your load-cell table |

## Model
```
I θ̈ = T L sin δ + m g h sin θ − bθ̇ − τc sgn θ̇ + τ_stop + τ_push − I_g δ̈
```
* Servo: position loop, then slew limit, velocity lag, torque limit, gimbal hard stop. It only latches a new command once per PWM frame (50 Hz analog vs 200–333 Hz digital).
* EDF: static map plus first-order spool-up. Throttle can be fixed or `@(t)`.
* IMU: gyro with noise and residual bias. Accelerometer specific force at `r_imu`, including the `rα` and `rω²` motion terms and EDF vibration, so the accelerometer-only angle is realistically wrong.
* Complementary filter, low-pass D-term, one-sample compute delay, command clamp.
* Human pilot: delayed, noisy PD with tremor, for the "why TVC needs a computer" comparison.

Not modeled: fan roll torque and rotor gyroscopic torque. In 1-DOF both act about axes the pivot reacts, so they load the bearings but don't move θ. Also not modeled: thrust loss vs gimbal angle and duct aerodynamics.

## Plugging in real data
* **Thrust stand (Nate):** `p.thrust_table = [u T; ...]`, `p.tau_edf` from a throttle-step test.
* **CAD (Alec/mech):** `p.m`, `p.h_cg`, `p.I`, `p.L`, and `p.Ig` from mass properties.
* **IMU bench (Warren):** `p.gyro_bias`, noise densities, and especially `p.accel_vib` measured with the EDF running.
* **Servo:** step-test it and fit `p.servo_kpos`/`p.servo_tau`. Set `p.servo_refresh` to what it actually accepts.

Then compare the `tvc_simulate` output to logged θ, θ̇, δ from the hardware (the project plan's model-vs-experiment requirement).
