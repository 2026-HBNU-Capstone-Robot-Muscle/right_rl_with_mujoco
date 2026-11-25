import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
r=json.loads((ROOT/'horizontal_quarter_trial_20261028_040503_642362/result.json').read_bytes())
print(json.dumps(dict(read_failures=r.get('read_failures'),confirmation_events=r.get('read_recovery_confirmation_events'),
    preflight_anomalies=r.get('preflight_static_read_anomalies'),
    min_voltage=min(v for s in r['samples'] for v in s['voltage_v']),
    peak_current=max(abs(v) for s in r['samples'] for v in s['current_ma']),
    max_tracking=max(v for s in r['samples'] for v in s['tracking_rad']),off=r['all_torque_off_verified'])))
import dynamixel_sdk,inspect
from dynamixel_sdk import protocol2_packet_handler as sdk_protocol
print('SDK path:',inspect.getfile(sdk_protocol))
print(inspect.getsource(sdk_protocol.Protocol2PacketHandler.rxPacket))
