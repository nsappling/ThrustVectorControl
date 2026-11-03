# flight_computer (Teensy 4.1)

The main controller: IMU → complementary filter → PD(+I) → HTD-30H gimbal servo, plus EDF throttle and the safety state machine. The control law matches `sim/tvc_sim/tvc_simulate.m`.

| File | What's in it |
|---|---|
| `flight_computer.ino` | State machine, 400 Hz loop, telemetry, serial commands |
| `config.h` | **Every pin, rate, limit, and default gain.** Pins must match `docs/pinout.md`. |
| `imu.*` | MPU-6050 driver and complementary filter |
| `servo_bus.*` | Hiwonder bus servo protocol (through the BusLinker) |
| `esc.*` | ESC PWM with the hard throttle cap |

## Setup
1. Install the Arduino IDE 2 and add Teensy support: in **Preferences → Additional boards manager URLs**, add `https://www.pjrc.com/teensy/package_teensy_index.json`. Then go to **Boards Manager → install "Teensy"**.
2. Open `flight_computer.ino` (the other files open as tabs). Set **Board: Teensy 4.1** and **USB Type: Serial**.
3. Upload. Open the Serial Monitor with line ending set to "Newline", and type `help`.

## Bring-up order (one piece at a time)
1. **IMU:** with the Teensy and IMU only, boot and check that `theta_deg` follows a hand tilt with the right sign (`IMU_SIGN`).
2. **Servo bus:** run `../tests/servo_bus_test` first: read the ID, voltage, and position, then center. Then in `flight_computer`, send `arm` and tilt the IMU by hand. **Direction check:** if the nose tips toward +x (θ > 0), the fan must tilt so its thrust pushes the nose back. If it goes the wrong way, flip `SERVO_DIR`.
3. **ESC/EDF:** run `../tests/esc_test` for throttle calibration and the current steps (see `docs/icd/edf_esc.md`).
4. **Closed loop:** `arm` → `throttle 0.15` → `run`. Have a hand on the E-stop. `stop` ends it.

## Commands
`stop`, `arm`, `run`, `throttle <0-1>`, `ref <deg>`, `kp/ki/kd <val>`, `trim <counts>`, `zero`, `status`, `hb`. Full table and CSV columns: `docs/icd/usb_link.md`.

## TODO
- Non-blocking servo reads. Each read currently blocks the loop for ~2 ms at 20 Hz.
- Have `tvc_logger.py` send `hb`, then enable `HOST_TIMEOUT_MS`.
- Joystick setpoints from the Pi (`ref`).
- Write the servo angle limits once the gimbal's hard stops are known.
