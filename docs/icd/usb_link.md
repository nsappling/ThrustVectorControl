# ICD: USB Link (Teensy ↔ Raspberry Pi 5)

**Endpoints:** Teensy micro-USB ↔ Pi USB-A, using a USB data cable. It also powers the Teensy and IMU.
**Status:** Preliminary

## Physical
- USB-A to micro-USB **data** cable, ≤ 1 m. Strain-relieve it where it crosses the pivot.
- The Teensy enumerates on Linux as `/dev/ttyACM0`. The baud setting is ignored (native USB).

## Teensy → Pi: telemetry
Text, one line per message, `\n` terminated.

| Line starts with | Meaning |
|---|---|
| `#` | Human-readable status or error. Print it, don't log it. |
| `t_ms,` | CSV header (sent once at boot) |
| anything else | CSV data row, 50 Hz |

CSV columns (`flight_computer`):

| Column | Unit | Description |
|---|---|---|
| t_ms | ms | Teensy uptime |
| state | — | 0 SAFE, 1 ARMED, 2 RUN, 3 FAULT |
| theta_deg | deg | Fused body angle |
| rate_dps | deg/s | Gyro rate (bias-corrected) |
| gimbal_cmd_deg | deg | Commanded fan tilt |
| gimbal_meas_deg | deg | Fan tilt read back from the servo (updated at 20 Hz) |
| throttle | 0–1 | EDF throttle command (after the cap) |
| servo_v | V | Servo input voltage (updated at 20 Hz) |

## Pi → Teensy: commands
Text, `\n` terminated, case-sensitive. The Teensy replies with a `#` line.

| Command | Effect |
|---|---|
| `help` | List commands |
| `status` | Print gains, state, limits |
| `arm` | SAFE → ARMED (ESC armed at zero throttle, servo holding) |
| `run` | ARMED → RUN (closed-loop control; EDF at the set throttle) |
| `stop` | Any state → SAFE (throttle 0, servo centered) |
| `throttle <0-1>` | Set EDF throttle (clamped to `THROTTLE_MAX`) |
| `ref <deg>` | Angle setpoint (later driven by the joystick) |
| `kp/ki/kd <val>` | Gains (not saved through power-off) |
| `trim <counts>` | Servo center offset, ±100 counts (0.24° per count) |
| `zero` | Re-calibrate the IMU (hold the rocket upright and still); not allowed in RUN |
| `hb` | Heartbeat. In RUN, if `HOST_TIMEOUT_MS` > 0 and no line arrives within it, the Teensy goes to SAFE. |

## Safety rules (enforced on the Teensy)
- Boots in **SAFE**. The EDF never spins until it receives `arm` and then `run`.
- Goes to **FAULT** (throttle 0, servo centered) on: IMU read failure, |θ| > `THETA_TRIP_DEG`, servo voltage out of range, or 10 missed servo replies in a row (in ARMED or RUN). `stop` clears FAULT back to SAFE.
- Unplugging USB removes Teensy power, which means no PWM. The ESC's signal-loss protection then stops the EDF.
