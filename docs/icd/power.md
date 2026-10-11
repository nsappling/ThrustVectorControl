# ICD: Power Distribution

**Endpoints:** wall outlet → AC inlet → PSU → F1 (EDF branch) / F2 (servo branch)
**Status:** Preliminary

## Connections

| # | From | To | Wire | Connector / termination | Notes |
|---|---|---|---|---|---|
| P1 | Wall | IEC C14 inlet (switch + fuse) | 3-wire grounded cord | IEC C13 cord | Inlet fuse T5A slow-blow (proposed) |
| P2 | Inlet L | PSU pin 1 (AC/L) | 18 AWG, brown/black | Insulated fork terminal | Switched and fused side |
| P3 | Inlet N | PSU pin 2 (AC/N) | 18 AWG, blue/white | Insulated fork terminal | |
| P4 | Inlet PE | PSU pin 3 (FG) | 18 AWG, green/yellow | Insulated fork terminal | **Required.** Earths the metal case. |
| P5 | PSU pin 7 (+V) | F1 15 A fuse | 12 AWG red | Ring/fork → inline blade-fuse holder | Keep under ~15 cm from the PSU |
| P6 | F1 | XT60 female (+) | 12 AWG red | Solder | Supply side gets the female |
| P7 | PSU pin 4 (−V) | XT60 female (−) | 12 AWG black | Ring/fork, solder | |
| P8 | PSU pin 8 (+V) | F2 5 A fuse | 18 AWG red | Ring/fork → inline blade-fuse holder | |
| P9 | F2 | BusLinker power terminal (+) | 18 AWG red | Screw terminal | |
| P10 | PSU pin 5 (−V) | BusLinker power terminal (−) | 18 AWG black | Screw terminal | Check polarity twice; the manual warns about reversal |

Pins 6 (−V) and 9 (+V) are spares. Each branch gets its own terminal pair, so neither branch's current passes through the other's screw.

## Electrical

| Parameter | Value |
|---|---|
| Bus voltage | 12.0 V nominal. **Don't raise it above 12.6 V** (servo max). |
| EDF branch | 15 A max (fuse) ≈ 180 W |
| Servo branch | 5 A max (fuse). Servo stalls at ~3 A. |
| PSU input switch | **115 V position** (US) |

## Safety

- The AC terminals on the PSU are exposed. Cover them, or mount the PSU in an enclosure, before anyone else is near it.
- Fuses go on the **+** side, as close to the PSU as practical.
- The PSU needs ~4 min to recover after a protection trip. The fan only runs above 50 °C internal, so leave the vents clear.
- E-stop: see open item 4 in [system_overview.md](../system_overview.md).

## Verification checklist
- [ ] Input switch set to 115 V **before** first plug-in
- [ ] Continuity from FG to the wall plug's earth pin
- [ ] No-load output reads 12.0 V ± 0.2 V at the PSU terminals
- [ ] Both fuses installed; polarity checked at the XT60 and BusLinker terminals with a meter
- [ ] Pull test on every crimp
