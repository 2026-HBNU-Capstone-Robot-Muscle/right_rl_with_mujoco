"""USB port enumeration only; does not open a port or issue a motor packet."""
import json
from serial.tools import list_ports
print(json.dumps(dict(ports=[dict(device=p.device,description=p.description,vid=p.vid,pid=p.pid) for p in list_ports.comports()],serial_ports_opened=False,motor_packets_sent=0),ensure_ascii=False,indent=2))
