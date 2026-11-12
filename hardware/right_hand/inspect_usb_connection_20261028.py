"""Enumerate host USB identities only; no motor port opened."""
import json
from serial.tools.list_ports import comports
print(json.dumps([dict(device=p.device,description=p.description,vid=p.vid,pid=p.pid,serial_number=p.serial_number,location=p.location) for p in comports()],ensure_ascii=False))
