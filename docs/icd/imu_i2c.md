# ICD: IMU (GY-521 / MPU-6050 ↔ Teensy)

**Endpoints:** GY-521 → Teensy Wire (pins 18/19)
**Status:** Preliminary

## Connections

| # | GY-521 pin | Teensy pin | Wire | Notes |
|---|---|---|---|---|
| I1 | VCC | 3.3V | 26 AWG | 3.3 V keeps the board's pull-ups at a Teensy-safe level |
| I2 | GND | GND | 26 AWG | |
| I3 | SCL | 19 (SCL) | 26 AWG | |
| I4 | SDA | 18 (SDA) | 26 AWG | |
| I5 | INT | 3 | 26 AWG | Optional, reserved for data-ready |
| — | AD0 | (open) | — | Address 0x68 |
| — | XDA, XCL | (open) | — | |

Keep the I2C wires short (< 30 cm) and twist SDA and SCL each with a GND wire. Route them away from the ESC motor leads. If the IMU has to go through the pivot on a long cable, use 100 kHz instead of 400 kHz.

## Configuration (set in firmware)

| Register | Value | Meaning |
|---|---|---|
| PWR_MGMT_1 (0x6B) | 0x01 | Wake, gyro X clock |
| CONFIG (0x1A) | 0x02 | DLPF ~94 Hz |
| GYRO_CONFIG (0x1B) | 0x08 | ±500 °/s (65.5 LSB per °/s) |
| ACCEL_CONFIG (0x1C) | 0x08 | ±4 g (8192 LSB per g) |
| Read | 14 bytes from 0x3B | accel XYZ, temp, gyro XYZ (big-endian int16) |

## Mounting
- Z axis along the rocket's long axis. Tilt in the gimbal plane is then rotation about the IMU's X axis (or Y; see `IMU_USE_Y_AXIS`).
- Mount it close to the pivot (less tangential-acceleration error) and on foam tape (less EDF vibration).
- Positive θ = nose tips toward +x, matching `tvc_params.m`. Set `IMU_SIGN` to match.

## Verification checklist
- [ ] `firmware/tests/hardware_test` (or `flight_computer`) finds the device at 0x68
- [ ] Angle sign matches the convention
- [ ] Noise with the EDF running at the throttle cap: ______ ° RMS (feeds `accel_vib` in `tvc_params.m`)
