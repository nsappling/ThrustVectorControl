# Docs

Start with **[system_overview.md](system_overview.md)**: block diagram, power budget, key decisions, and open items.

| Doc | What it answers |
|---|---|
| [system_overview.md](system_overview.md) | How it all fits together, and what's still undecided |
| [components.md](components.md) | Specs and sources for every part (and how confident we are in each number) |
| [pinout.md](pinout.md) | What's on every Teensy pin (single source of truth) |
| [icd/power.md](icd/power.md) | AC inlet, PSU, fuses, wire gauges, branch wiring |
| [icd/edf_esc.md](icd/edf_esc.md) | ESC ↔ EDF, throttle signal, throttle cap |
| [icd/servo_bus.md](icd/servo_bus.md) | Teensy ↔ BusLinker ↔ HTD-30H, packet protocol |
| [icd/imu_i2c.md](icd/imu_i2c.md) | GY-521 wiring and register setup |
| [icd/usb_link.md](icd/usb_link.md) | Teensy ↔ Pi: telemetry format and commands |
| [wiring/](wiring/) | Harness diagram: `tvc_harness.svg` / `.png`, with a BOM in `.bom.tsv` |
| `datasheets/` | Local copies of the datasheets we rely on |
| `sketches/` | Hand sketches |

## Conventions
- **One interface, one ICD.** Each ICD has a connection table (wire IDs like `P5` and `S1` also appear on the harness diagram), the electrical and protocol specs, and a verification checklist. Tick the checklist boxes as you test.
- **[VERIFY]** marks a number that isn't confirmed yet. When you measure it, replace the tag with the value and the date.
- **Change the docs in the same commit as the hardware or firmware change.** If you move a wire, update `pinout.md` and the relevant ICD, then regenerate the harness diagram.

## Regenerating the harness diagram
```bash
pip install wireviz        # also needs Graphviz (brew install graphviz / apt install graphviz)
wireviz docs/wiring/tvc_harness.yml
```
