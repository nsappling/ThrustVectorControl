# Teensy 4.1 Pin Assignments

This is the single source of truth. `firmware/flight_computer/config.h` must match this table. If you change a pin, change both files in the same commit.

| Teensy pin | Name in `config.h` | Direction | Connects to | Level | Interface doc |
|---|---|---|---|---|---|
| 0 (RX1) | `SERVO_BUS_SERIAL` | in | Level shifter LV1 → BusLinker **TX** | 3.3 V | [servo_bus](icd/servo_bus.md) |
| 1 (TX1) | `SERVO_BUS_SERIAL` | out | Level shifter LV2 → BusLinker **RX** | 3.3 V | [servo_bus](icd/servo_bus.md) |
| 2 | `PIN_ESC` | out | ESC signal (white/yellow) | 3.3 V PWM | [edf_esc](icd/edf_esc.md) |
| 3 | `PIN_IMU_INT` | in | GY-521 INT (optional, reserved) | 3.3 V | [imu_i2c](icd/imu_i2c.md) |
| 13 | `PIN_LED` | out | Onboard LED (status) | — | — |
| 18 (SDA) | `Wire` | bidir | GY-521 SDA | 3.3 V | [imu_i2c](icd/imu_i2c.md) |
| 19 (SCL) | `Wire` | out | GY-521 SCL | 3.3 V | [imu_i2c](icd/imu_i2c.md) |
| 3.3V | — | power out | GY-521 VCC, level shifter LV | 3.3 V | |
| VIN | — | power out* | Level shifter HV | 5 V (from USB) | |
| GND | — | — | GY-521 GND, level shifter GND, ESC signal GND, BusLinker GND | | |
| USB | `Serial` | bidir | Raspberry Pi 5 | — | [usb_link](icd/usb_link.md) |

\* VIN is joined to USB 5 V on the board. Never connect another 5 V source (such as the ESC's BEC) to VIN while USB is plugged in.

## Reserved for later
| Pin | Planned use |
|---|---|
| 4 | Arm switch / E-stop sense input (pull-up, active low) |
| 14 (A0), 15 (A1) | Analog joystick, if it ever moves to the Teensy |
| 7 (RX2), 8 (TX2) | Spare UART |
