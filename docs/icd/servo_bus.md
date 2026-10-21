# ICD: Servo Bus (Teensy ↔ BusLinker ↔ HTD-30H)

**Endpoints:** Teensy Serial1 → level shifter → BusLinker TTL header → servo bus → HTD-30H (ID 1)
**Status:** Preliminary. The BusLinker header pinout and logic level still need checking.

## Connections

| # | From | To | Wire | Notes |
|---|---|---|---|---|
| S1 | Teensy pin 1 (TX1) | Level shifter LV2 | 26 AWG | |
| S2 | Level shifter HV2 | BusLinker **RX** | 26 AWG | |
| S3 | BusLinker **TX** | Level shifter HV1 | 26 AWG | |
| S4 | Level shifter LV1 | Teensy pin 0 (RX1) | 26 AWG | |
| S5 | Teensy 3.3V | Level shifter LV | 26 AWG | |
| S6 | Teensy VIN (5 V) | Level shifter HV | 26 AWG | |
| S7 | Teensy GND | Level shifter GND + BusLinker GND | 26 AWG | |
| S8 | BusLinker servo port (PH2.0) | HTD-30H | Supplied cable | Carries 12 V, GND, and the data line |
| — | BusLinker power terminal | Servo branch (F2) | 18 AWG | See [power.md](power.md) |

**BusLinker jumpers:** **Servo + TTL** when the Teensy is in control. Use **Servo + USB** only when setting up the servo from the Hiwonder PC software (for example, changing the ID or offset).

### Why the level shifter
The protocol doc's reference circuit pulls the bus up to **5 V**, and the Teensy 4.1 is **not 5 V tolerant**. Until we know the BusLinker header's logic level, assume 5 V and use the shifter. If you power the BusLinker and measure its idle TX pin at ≤ 3.3 V, the shifter can be dropped. Record that result here.

## Protocol (per [Hiwonder protocol doc](../datasheets/Hiwonder_bus_servo_protocol.pdf))

- 115200 baud, 8N1, half-duplex. The BusLinker handles direction switching.
- Packet: `0x55 0x55 | ID | LEN | CMD | params... | CHK`
  - `LEN` = number of params + 3
  - `CHK` = `~(ID + LEN + CMD + params) & 0xFF`
- Little-endian 16-bit values. Position 0–1000 = 0–240° (center 500).

| Command | CMD | Params | Reply |
|---|---|---|---|
| MOVE_TIME_WRITE | 1 | pos (u16), time ms (u16) | — |
| POS_READ | 28 | — | pos (s16) |
| VIN_READ | 27 | — | mV (u16) |
| TEMP_READ | 26 | — | °C (u8) |
| ANGLE_LIMIT_WRITE | 20 | min (u16), max (u16). Saved through power-off. | — |
| LOAD_OR_UNLOAD_WRITE | 31 | 0 = limp, 1 = torque on | — |
| ID_READ | 14 | — (use ID 254 with only one servo on the bus) | id (u8) |

Example: move servo 1 to center (500) in 0 ms → `55 55 01 07 01 F4 01 00 00 01` (the last byte is the checksum).

## Timing
- One move packet is 10 bytes ≈ 0.87 ms at 115200 baud. The firmware sends moves at 100 Hz.
- A read (request + reply) takes ~1.5–2 ms, so position feedback is read at a lower rate (20 Hz).
- The servo's internal response time is unknown [VERIFY with a step test]. It feeds `servo_tau` in `tvc_params.m`.

## Mechanical limits
Once the gimbal is built, write `ANGLE_LIMIT_WRITE` so the servo itself can't drive the gimbal into its hard stops. The firmware also clamps the command (`GIMBAL_MAX_DEG`).

## Verification checklist
- [ ] BusLinker header pin labels recorded: ______
- [ ] Idle TX voltage measured: ______ V
- [ ] `firmware/tests/servo_bus_test` reads the ID, voltage (≈ 12 V), and position
- [ ] Center (500) means gimbal straight; trim recorded: ______ counts
- [ ] Direction: positive gimbal command tilts the fan the way `SERVO_DIR` expects
