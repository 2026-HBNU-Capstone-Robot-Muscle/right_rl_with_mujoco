"""Apply the explicit user request once, preserving the original reference."""
import hashlib,json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parent;base=root/'right_hand_rotation_20261023/native_motor_limits.json'
path=root/'loaded_quarter_id4_lower_setting_20261025.json';blob=base.read_bytes();r=json.loads(blob)
lower=float(r['lower_ticks'][3]);new=lower-np.deg2rad(2.)/r['tick_rad']
if path.exists():raise RuntimeError('Setting already exists; do not apply twice')
setting=dict(version='user_ID4_lower_minus_two_degrees_v1',requested_client_date='2026-10-25',motor_id=4,
    lower_degrees_by=2.,base_limits_source=base.relative_to(root).as_posix(),base_limits_sha256=hashlib.sha256(blob).hexdigest(),
    original_lower_ticks=lower,configured_lower_ticks=new,original_minimum_integer_ticks=int(np.ceil(lower)),
    configured_minimum_integer_ticks=int(np.ceil(new)),other_joint_limits_unchanged=True,upper_limits_unchanged=True,
    scope='current supervised loaded-quarter command and its model',hardware_eeprom_modified=False,hardware_commands_sent=0)
path.write_text(json.dumps(setting,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(setting,ensure_ascii=False))
