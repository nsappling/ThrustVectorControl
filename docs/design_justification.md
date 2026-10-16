# Design Justification: What Each Component Has to Satisfy

A short list of the considerations behind each part choice, to show *why* each part was picked. Numbers marked [EST] are placeholders from `sim/tvc_sim/tvc_params.m` until measured. Specs and sources are in [components.md](components.md).

## System-level requirements (these couple the parts together)

- **Control authority, not thrust-to-weight.** The vehicle is on a pivot, so thrust never has to lift it (T/W > 1 isn't required). What matters is that the gimbal can out-torque gravity: **T · L · sin(δmax) > m · g · h · sin(θstop)**. Here T = thrust, L = pivot-to-gimbal distance, δmax = maximum gimbal angle, m = rotating mass, h = pivot-to-CG height, θstop = tilt at the hard stops. Higher thrust or a longer lever arm helps. More mass or a higher CG hurts. `tvc_linear_report.m` computes this ratio.
- **Mass distribution about the pivot.** The CG height above the pivot (h) and the inertia about the pivot (I) set how fast the rocket falls over: unstable pole = √(m g h / I) ≈ 4 rad/s [EST]. A lower CG or more inertia makes it slower and easier to control, but less dramatic as a demo. Keep pivot friction and cable drag across the pivot low, or the controller will hunt back and forth around upright.
- **Closed-loop bandwidth.** It should be ~3–5× the unstable pole. The current design targets ωn = 12 rad/s. The control loop runs at 400 Hz, far faster than that.
- **Servo response.** The gimbal has to move noticeably faster than the closed loop: a small (~5–15°) step should settle in well under ~50 ms. Lag, backlash, and deadband all eat into the phase margin.
- **Gimbal torque.** Servo torque has to cover the EDF/cradle inertia times the required gimbal acceleration, plus any offset between the thrust line or EDF CG and the gimbal axis, with margin. A heavier EDF adds inertia, which needs more servo torque.
- **Electrical compatibility.** Supply voltages and logic levels have to match across every connection. Each protocol needs enough rate for its loop.
- **Overcurrent protection.** Each power branch is fused below its wire's rating and above the load's normal current.
- **Exhibit constraints.** It has to run all day (mains power, no batteries), keep noise down (~70–75 dB target), be safe around kids (guard, E-stop, enclosure), and stay on budget.

## Components

### EDF: [JFtech 64 mm 12-blade 4300 KV (3S)](https://www.amazon.com/dp/B0F9N6YCX9)
- Thrust at the operating current has to give enough control authority. Estimate: 5–7.5 N at 15 A [VERIFY on the thrust stand].
- Mass and size feed the inertia the servo has to move (Ig) and the vehicle's mass and CG. The 64 mm fan fits inside the 3.5 in body tube.
- The rated voltage has to match the supply: 3S ≈ 11.1–12.6 V, so it runs on the 12 V PSU directly.
- Current draw has to fit the ESC, fuse, and wiring: ~50 A at full throttle, so the throttle is capped in firmware.
- Spool-up time and vibration: vibration corrupts the IMU's accelerometer, so mount the IMU on foam and measure the noise with the fan running.
- Noise limit for the exhibit.

### ESC: [Hobbywing Skywalker 50A V2](https://www.hobbywingdirect.com/products/skywalker-esc-50a)
- Continuous current has to exceed the EDF's operating current: 50 A continuous / 70 A peak vs. 15 A fused. The seller recommends 80 A for this EDF at full throttle, so revisit if the fuse is ever raised.
- Input voltage range has to include 12 V: rated 3–4S ✅.
- Throttle input: standard 50 Hz PWM. It must accept the Teensy's 3.3 V logic [VERIFY].
- Safety: it stops the motor on signal loss ✅. Disable the brake. The built-in 5 V BEC isn't needed.
- Cost: ~$25.

### Gimbal servo: [Hiwonder HTD-30H](https://www.hiwonder.com/products/htd-30h)
- **Torque:** 2.94 N·m stall (30 kg·cm at 12.6 V), which met the early 2–3 N·m target from rough gimbal inertia and response-time sizing. That target is a placeholder. The sim (`tvc_servo_sizing.m`) puts the usable torque needed at the gimbal nearer ~0.2 N·m. Confirm the margin with the real EDF/cradle inertia and the CG offset from the gimbal axis.
- **Speed:** 0.12 s/60°, comfortably inside the ≤ 0.2 s/60° limit, so it adds little lag to the closed loop.
- **Feedback:** reports position, voltage, and temperature over its bus, so we can log the actual gimbal angle (for anti-windup and system ID). It doesn't report current or load.
- **Interface:** 115200-baud serial bus, plenty for a 100–200 Hz command rate plus slower telemetry reads.
- **Power and weight:** 9–12.6 V, so it runs straight off the 12 V supply. ~3 A stall, under the 5 A fuse. 63 g, a light addition to the gimbal.
- **Budget:** ~$20, under the $100 ceiling, with room for a spare. That ruled out the KST X20 (~$200) and the Dynamixel XM430 ($240–310).
- **Alternative considered:** the HTD-85H (8.34 N·m stall), which has more torque margin but is slower (0.2 s/60°) and has an unknown stall current.

### Servo bus adapter: [Hiwonder BusLinker V3.0](https://www.hiwonder.com/products/buslinker)
- Converts the Teensy's 2-wire UART into the servo's 1-wire half-duplex bus.
- Input range 5–14 V covers 12 V ✅. It has the HTD-30H's PH2.0 connector ✅.
- Logic level on its serial header is unknown [VERIFY], hence the level shifter. Its current rating through the board is also unpublished [VERIFY against the 3 A servo stall current].
- It also allows PC setup of the servo (ID, offsets, limits) over USB.

### Level shifter: BSS138 4-channel, 3.3 ↔ 5 V
- The Teensy isn't 5 V tolerant, and the servo bus idles at 5 V.
- It has to pass 115200 baud cleanly; the BSS138 is good to well beyond that ✅.

### IMU: [GY-521 (MPU-6050)](https://www.amazon.com/dp/B07PY38BCB)
- Gyro range has to cover the expected tilt rates: ±500 °/s used ✅.
- Noise and drift: handled by fusing gyro and accelerometer in a complementary filter. Bias is calibrated at startup.
- Vibration sensitivity: the EDF is the biggest noise source. Mount near the pivot and on foam.
- I2C at 400 kHz, powered from 3.3 V so its pull-ups stay Teensy-safe ✅.
- Cheap and well documented. If vibration noise turns out to be bad, upgrade to an ICM-42688 or BNO085.

### Flight computer: [Teensy 4.0](https://www.pjrc.com/store/teensy40.html)
- Needs plenty of headroom for a 400 Hz loop: 600 MHz Cortex-M7 ✅.
- Needs the right peripherals: UART (servo), I2C (IMU), PWM (ESC), native USB (Pi) ✅.
- 3.3 V logic, which forces level shifting on the servo bus.
- Small and light if it rides on the rocket. Arduino IDE support, so the team's workflow carries over from the Uno. ~$24.

### Ground computer: [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/)
- Kept out of the real-time loop: it only handles logging, tuning, the joystick, and the exhibit display.
- USB host for the Teensy and a gamepad. Runs Python for logging and the UI.
- Powered by its own 5 V 5 A supply, so motor noise doesn't reset it.

### Power supply: [Mean Well LRS-600-12](https://www.bravoelectro.com/lrs-600-12.html)
- Mains power instead of LiPo, because the exhibit runs all day with no battery charging or fire risk.
- Voltage matches the 3S EDF and the servo's ≤ 12.6 V limit: 12.0 V nominal (adjustable 11.4–13.2 V) ✅.
- Current capacity (50 A) is well above the fused branch total (≤ 20 A) ✅.
- Wall draw at our ≤ 250 W is ~2.5 A AC, fine for a standard outlet.
- Built-in short-circuit, overload, overvoltage, and over-temperature protection. It needs earth ground and covered AC terminals.

### Fuses: F1 15 A (EDF branch), F2 5 A (servo branch), inline blade
- Rating has to be above the normal load current and below the wire's ampacity.
- F1 also acts as a power limit on the EDF (~180 W). The firmware throttle cap keeps normal operation under it.
- F2 has to be above the servo's stall current (~3 A for the HTD-30H) ✅.
- Mounted on the + side, close to the PSU.

### Wiring and connectors
- Wire ampacity has to exceed the fuse rating: 12 AWG (EDF) ≫ 15 A and 18 AWG (servo) > 5 A ✅. Keep voltage drop small at peak current.
- Connector ratings: XT60 ≈ 30 A continuous and 3.5 mm bullets ≈ 60 A, both ≫ 15 A ✅.
- Use silicone wire for flexibility. Add strain relief and a low-drag cable path anywhere wiring crosses the pivot.
- Signal wiring: short, twisted I2C pairs, routed away from the ESC's motor leads.

### Safety items (to order): E-stop, fused IEC inlet, watt meter
- A latching mains E-stop cuts everything at once.
- A fused, switched inlet protects the AC side.
- An inline watt meter is needed to set the throttle cap safely.
