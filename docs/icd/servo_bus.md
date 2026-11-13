# ICD: Servo Bus (Teensy ↔ BusLinker ↔ HTD-30H)

**Endpoints:** Teensy Serial2 → BusLinker TTL header (direct, no level shifter) → servo bus → HTD-30H (ID 1)
**Status:** Working on the bench (team bring-up). The BusLinker TX voltage into the Teensy still needs measuring; see below.

## Connections

| # | From | To | Wire | Notes |
|---|---|---|---|---|
| S1 | Teensy pin 8 (TX2) | BusLinker **RX** | 26 AWG | 3.3 V out. Works on the bench. |
| S3 | BusLinker **TX** | Teensy pin 7 (RX2) | 26 AWG | ⚠️ Must be ≤ 3.3 V; see below |
| S7 | Teensy GND | BusLinker GND | 26 AWG | Required |
| S8 | BusLinker servo port (PH2.0) | HTD-30H | Supplied cable | Carries 12 V, GND, and the data line |
| — | BusLinker power terminal | Servo branch (F2) | 18 AWG | See [power.md](power.md) |

**BusLinker jumpers:** **Servo + TTL** when the Teensy is in control. Use **Servo + USB** only when setting up the servo from the Hiwonder PC software (for example, changing the ID or offset).

### Check: is BusLinker TX safe for the Teensy?
We run without a level shifter. The Teensy 4.1's pins are **not 5 V tolerant**, and the protocol doc's reference circuit pulls the bus up to 5 V, so measure this once:

1. Power the BusLinker (12 V on its terminal), **unplug the TX wire from the Teensy**, and measure BusLinker TX to GND with a meter while the bus is idle.
2. **≤ 3.3 V:** direct wiring is fine. Record the reading below.
3. **Around 5 V:** add a resistor divider on S3: BusLinker TX → **1 kΩ** → Teensy pin 7, and pin 7 → **2 kΩ** → GND. That gives about 3.3 V. Two resistors, no new parts to order; fine at 115200 baud.

Teensy TX (3.3 V) into BusLinker RX is fine either way: it already works.

Measured idle TX voltage: ______ V (date: ______)

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
- [ ] Idle TX voltage measured (above): ______ V
- [ ] `firmware/tests/servo_bus_test` reads the ID, voltage (≈ 12 V), and position
- [ ] Center (500) means gimbal straight; trim recorded: ______ counts
- [ ] Direction: positive gimbal command tilts the fan the way `SERVO_DIR` expects
