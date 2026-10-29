# Components

Confidence: **DS** = manufacturer datasheet in `datasheets/`. **MFR** = manufacturer product page. **SELLER** = reseller or Amazon listing. **[VERIFY]** = unconfirmed, so measure or check on receipt.

## Power supply: Mean Well LRS-600-12 (DS)
[Product page](https://www.bravoelectro.com/lrs-600-12.html) · [datasheet](datasheets/MeanWell_LRS-600_spec.pdf)

| Spec | Value |
|---|---|
| Output | 12 V, 0–50 A, 600 W. Adjustable 11.4–13.2 V (pot next to the LED). |
| Ripple | 200 mVp-p max |
| Input | **90–132 VAC or 180–264 VAC, set by a slide switch. Set it to 115 V in the US.** |
| AC current / inrush | 12 A at 115 VAC (full 600 W load) / 35 A inrush at 115 VAC |
| Protection | Overload at 105–150 % (constant current, shuts down after 3 s). Overvoltage at 13.8–16.2 V. Over-temperature. Recovery needs a power cycle, then a ~4 min cool-down. |
| Terminals | 1 = AC L, 2 = AC N, 3 = FG (earth), **4–6 = −V, 7–9 = +V** (screw terminals) |
| Size / mass | 225 × 124 × 41 mm, 0.95 kg. The fan turns on above 50 °C internal. |

## ESC: Hobbywing Skywalker 50A V2
[Product page](https://www.hobbywingdirect.com/products/skywalker-esc-50a) (MFR). The spec chart there is an image; values below are from resellers.

| Spec | Value |
|---|---|
| Current | 50 A continuous, 70 A peak (SELLER) |
| Input | 3–4S LiPo (≈ 9.9–16.8 V), so 12 V is fine (SELLER) |
| BEC | 5 V / 5 A switching (SELLER). **Not used in our design.** |
| Throttle input | Standard RC PWM: 50 Hz, 1000–2000 µs [VERIFY in manual; do throttle-range calibration] |
| Features | 32-bit MCU, brake modes, throttle-signal-loss protection, programmable with the LED program box |
| Size / mass | 60 × 25 × 8 mm, 36 g (SELLER) |
| Connectors | Input and motor-lead connectors [VERIFY on receipt]. We add an XT60 on the input and 3.5 mm bullets on the motor side. |

## EDF: JFtech 64 mm, 12-blade, 4300 KV, 3S version
[Amazon](https://www.amazon.com/dp/B0F9N6YCX9) (SELLER)

| Spec | Value |
|---|---|
| Fan / motor | 64 mm, 12 blades, 2822-size 4300 KV brushless motor, CW |
| Voltage | 3S (11.1 V nominal), so 12 V is fine |
| Claimed | 1810 g max thrust, ~720 W, 80 A ESC recommended (SELLER, optimistic) |
| Comparable unit | QX-Motor 64 mm 4300 KV: **~1150 g at 53 A, 11.1 V** [VERIFY] |
| At our 15 A fuse | ≈ 500–780 g (5–7.5 N), from fan laws (current ∝ rpm³, thrust ∝ rpm²) [VERIFY on thrust stand] |
| Connector | 3.5 mm bullets (typical) [VERIFY] |

## Gimbal servo: Hiwonder HTD-30H
[Product page](https://www.hiwonder.com/products/htd-30h) · [specs (reseller)](https://openelab.io/products/htd-30h-bus-servo) · [protocol](datasheets/Hiwonder_bus_servo_protocol.pdf)

| Spec | Value |
|---|---|
| Voltage | **9–12.6 V**, so it runs straight off 12 V |
| Torque / speed | 30 kg·cm stall at 12.6 V, 0.12 s/60° |
| Stall current | ~3 A |
| Range | 0–240° (position mode), or continuous rotation (motor mode) |
| Command resolution | 0–1000 counts = 0–240° (0.24°/count). Listed accuracy 0.3°. |
| Interface | **Half-duplex TTL serial bus, 115200 baud**, 1 signal wire with a 10 kΩ pull-up to 5 V on the controller side (per the protocol doc). Two ports for daisy-chaining. |
| Feedback | Position, input voltage, temperature. Stall and over-temperature protection. |
| ID | 0–253, default **1**. 254 = broadcast. |
| Connector | PH2.0 3-pin, 200 mm cable included. Pin order: use the supplied cable [VERIFY]. |
| Size / mass | 40.0 × 20.1 × 51.1 mm, 63 g |

Some Hiwonder docs show 1 Mbaud and 4096-count positions. Those are for their newer HX-series servos. **The HTD-30H uses the older protocol described here** (0x55 0x55 header, 0–1000 positions, 115200 baud) [VERIFY with a ping on first power-up].

## Servo bus adapter: Hiwonder BusLinker V3.0
[Product page](https://www.hiwonder.com/products/buslinker) · [manual](https://docs.hiwonder.com/projects/BusLinker/en/latest/)

| Spec | Value |
|---|---|
| Power input | **5–14 V DC**, via a 5.5 × 2.1 mm barrel jack or a screw terminal. The input passes through to the servo ports. |
| Interfaces | USB-C (PC software), TTL serial header (external MCU), bus-servo ports (5264, PH2.0, 1.25T) |
| Mode jumpers | **Servo + TTL** = controlled by the Teensy. **Servo + USB** = controlled by the PC software. |
| Serial header | Pin labels and logic level not published [VERIFY: read the silkscreen and measure the idle TX voltage] |
| Size | 42 × 33 × 15 mm, 10 g |

## IMU: GY-521 (MPU-6050)
[Amazon](https://www.amazon.com/dp/B07PY38BCB) · [MPU-6050 datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)

| Spec | Value |
|---|---|
| Supply | VCC 3–5 V, with an onboard 3.3 V LDO |
| Interface | I2C up to 400 kHz. Address **0x68** (AD0 low/open) or 0x69 (AD0 high). Onboard pull-ups to the LDO output. |
| Pins | VCC, GND, SCL, SDA, XDA, XCL, AD0, INT |
| Ranges | Gyro ±250/500/1000/2000 °/s. Accel ±2/4/8/16 g. We use ±500 °/s and ±4 g. |

## Flight computer: PJRC Teensy 4.1
[Pinout and docs](https://www.pjrc.com/store/teensy41.html)

| Spec | Value |
|---|---|
| MCU | iMXRT1062, 600 MHz Cortex-M7 |
| Logic | **3.3 V. Pins are NOT 5 V tolerant.** |
| Power | VIN 3.6–5.5 V. VIN and VUSB are joined by a cuttable pad. The 3.3 V pin supplies ~250 mA. |
| USB | Micro-USB, native 480 Mbit/s serial (the baud setting is ignored) |
| Memory | 8 MB flash, 1 MB RAM, microSD slot (could log onboard if ever needed) |
| Size | 61 × 18 mm. Pins 0–23 sit where they do on a Teensy 4.0, so the pin table carries over. |
| Used peripherals | Serial2 (pins 7/8), Wire (pins 18/19), PWM (pin 2) |

## Ground computer: Raspberry Pi 5
- Power: official 5 V 5 A USB-C supply.
- Connects to the Teensy with a USB-A to micro-USB **data** cable (some cheap cables are power-only).

## Wiring and connectors

| Item | Use |
|---|---|
| 12 AWG silicone wire | PSU → F1 → XT60 → ESC (EDF branch) |
| 18 AWG silicone wire | PSU → F2 → BusLinker (servo branch). AC mains wiring is 18 AWG minimum (3-conductor). |
| 22–26 AWG / Dupont / JST | Signals: I2C, UART, ESC throttle |
| XT60 | EDF branch disconnect (female on the supply side) |
| 3.5 mm bullets | ESC ↔ EDF motor phases |
| Inline blade-fuse holders | F1: 15 A (12 AWG holder). F2: 5 A (18 AWG holder). |
| Fork/ring terminals | All PSU screw-terminal connections; no bare wire under the screws |

## Recommended additions (not yet ordered)
- **4-channel BSS138 level shifter** (3.3 V ↔ 5 V) for the Teensy ↔ BusLinker UART. About $1.
- **Inline XT60 watt meter** to set the throttle cap and log EDF current.
- **IEC C14 inlet with switch and fuse holder**, plus a 3-wire grounded cord.
- **Latching E-stop**, mains-rated, on the AC input.
- Clear cover or enclosure over the PSU's AC terminals.
