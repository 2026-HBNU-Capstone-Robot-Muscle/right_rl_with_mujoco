"""Read-only Windows serial-port inventory; never opens a port."""

import json
from serial.tools.list_ports import comports

print(json.dumps([{
    "device": p.device, "description": p.description, "hwid": p.hwid,
    "vid": p.vid, "pid": p.pid, "serial_number": p.serial_number,
} for p in comports()], ensure_ascii=False, indent=2))
