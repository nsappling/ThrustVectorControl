# TVC controller: hardware setup and bench test

`tvc_controller.ino` is the Arduino version (SparkFun RedBoard, Uno-compatible) of the AUTO-mode controller in `rocketgimball.m` / `tvc_sim.m`: the same low-pass filter, PID, ±35° saturation, and fan-tilt sign convention.

## System layout

```
 GY-521 (MPU-6050) --I2C--> RedBoard   --PWM--> 20 kg servo --> fan gimbal
                             |    USB
                        Raspberry Pi 5  (logging, tuning; later: joystick / exhibit UI)
```

- **RedBoard: real-time control.** It runs the loop at 500 Hz with steady timing, which the PID math relies on.
- **Pi 5: supervisor only.** It's connected over USB, logs data, and sends gain changes. It doesn't need to be connected for the controller to run. Don't wire the RedBoard's TX/RX pins directly to the Pi's GPIO: the Pi uses 3.3 V and the RedBoard uses 5 V. USB is simpler and safe.

## Wiring

| From | To | Notes |
|---|---|---|
| GY-521 VCC | 5V | The GY-521 has its own 3.3 V regulator. |
| GY-521 GND | GND | |
| GY-521 SDA | SDA pin (next to AREF) | Internally the same as A4. |
| GY-521 SCL | SCL pin (next to AREF) | Internally the same as A5. |
| Servo signal (orange/yellow) | D4 | Set by `SERVO_PIN`. |
| Servo + (red) | **External 6 V supply +** | 5–7.4 V. It needs about 3 A capacity, since stall current is ~2.5 A. |
| Servo – (brown/black) | External supply – | |
| External supply – | RedBoard GND | **Required.** Without a common ground the servo signal has no reference. |

⚠️ **Never power the servo from the RedBoard's 5V pin.** A 20 kg servo can pull amps when it moves. That will brown out or reset the RedBoard, or damage the USB port or the Pi. Use a 6 V BEC, a bench supply, or a 5 V ≥3 A adapter.

Optionally, put a 470–1000 µF capacitor across the servo supply near the servo to absorb current spikes.

## IMU mounting

Mount the GY-521 with its **Z axis along the rocket's long axis**, chip side facing the nose, so tilting in the gimbal plane rotates the board about its X axis.
- If the gimbal plane lines up with the board's Y axis instead, set `IMU_USE_Y_AXIS = true`.
- Mount it firmly on foam tape or standoffs. Fan vibration is the main noise source for the accelerometer.

## Setup

1. Open `tvc_controller.ino` in the Arduino IDE.
2. Board: **Arduino Uno** (the RedBoard is Uno-compatible). Port: the `usbserial` / CH340 one. If the port does not appear, install the CH340 driver from SparkFun.
3. Check the settings at the top of the sketch:
   - `SERVO_RANGE_DEG`: 180 or 270, depending on your MIUZEI model.
   - `GIMBAL_RATIO`: 1.0 if the servo horn drives the gimbal directly.
4. Upload, then open Serial Monitor at 115200 baud with line ending set to "Newline". Hold the IMU **upright and still** for about 2 s while it calibrates.

## Bench test (no fan): tilt the IMU by hand to fake the body angle

Do these checks **in order**. Keep the fan unpowered.

1. **Angle sign.** Tip the IMU so the rocket's nose leans RIGHT. `theta_deg` should go **positive**, matching MATLAB's convention. If it goes negative, set `IMU_SIGN = -1.0`.
2. **Servo center.** Send `center`. The fan should point straight along the body. If it doesn't, adjust with `trim 20` (µs; try ±values) or re-seat the servo horn. Send `run` to resume.
3. **Servo direction (the critical one).** Lean the nose RIGHT. The fan's **exhaust should swing LEFT**, pointing opposite to the lean. That pushes the tail right and brings the nose back upright. If the exhaust swings the same way as the lean, set `SERVO_DIR = -1.0`. A wrong sign here is positive feedback, and the real rocket will slam into its stops.
4. **Response.** Tip it quickly and hold it. The fan should snap toward full correction. Hold at ~10° for a few seconds: the fan tilt slowly grows as the integral term builds, then caps at ±35°. This is expected, because in the bench test the servo can't actually push the IMU back upright.
5. **Noise.** Hold it still and upright. The fan should sit near 0° with small jitter. Large chatter means you should lower Kd or raise `tau`.

## Tuning from the Pi (or a laptop)

```bash
pip install pyserial
python3 tvc_logger.py
```

Type commands such as `kp 0.5`, `kd 0.035`, `tau 0.02`, `zero`, `center`, `run`, or `help`. They take effect immediately, but aren't saved when the board powers off. Copy any gains you settle on into the sketch.

Each run is saved as `tvc_log_YYYYMMDD_HHMMSS.csv`. To load a log in MATLAB and compare it to the simulation:

```matlab
d = readtable('tvc_log_....csv');
plot(d.t_ms/1000, d.theta_filt_deg)
```

## Differences from the MATLAB sim

| | MATLAB | RedBoard | Why |
|---|---|---|---|
| Angle measurement | `theta + noise` | Complementary filter of gyro + accel (`COMP_TAU` = 0.5 s) | A real sensor needs fusion: the accelerometer is noisy, and the gyro drifts. |
| Loop rate | 2000 Hz physics step | 500 Hz control loop | The gains transfer unchanged because the PID math uses `dt` explicitly. |
| Integral | No limit | Anti-windup: stops growing while the servo is pinned at its limit | Bench testing holds large errors for seconds at a time. |
| Safety | Mechanical stops | Servo centers if the angle is past 45° (resumes automatically below 35°) or the IMU read fails | |
| Servo | Instant | Servo pulse updates at 50 Hz, plus the servo's own speed | Worth adding a servo lag to `tvc_sim.m` later. |

## Before running with the fan

- Measure the real `F_t`, `R`, `m`, and `r_0`. Re-run `tune_pid.m` with those values, then put the new gains in the sketch.
- Secure the fan and add a guard. Have a physical kill switch on the EDF power.
