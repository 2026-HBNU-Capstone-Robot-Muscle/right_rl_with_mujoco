from serial.tools.list_ports import comports
import json
print(json.dumps([dict(device=p.device,description=p.description,vid=p.vid,pid=p.pid,serial=p.serial_number) for p in comports()]))
