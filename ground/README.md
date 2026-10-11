# Ground computer

Code that runs on the Raspberry Pi 5 (or any laptop) connected to the flight computer over USB.

| File | What it does |
|---|---|
| `tvc_logger.py` | Logs the controller's CSV telemetry to a file and sends tuning commands (`kp 0.5`, `center`, `help`, ...). |

```bash
pip install pyserial
python3 ground/tvc_logger.py
```
