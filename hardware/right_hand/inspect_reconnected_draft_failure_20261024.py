"""Read failed offline geometry and select an earlier unpaired frame."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'right_hand_rotation_20261024/grasped_yaw_reconnected_readonly_fixture'
r=json.loads((folder/'draft_diagnostic_only.json').read_bytes())
print('DRAFT_NOMINAL',json.dumps(r['static_screens'][0]))
print('DRAFT_MAX_SUPPORT',max(s['initial_support_overlap_mm'] for s in r['static_screens']))
photo=ROOT/'camera_readonly_check_20261024_042734_377020/camera'
frames=json.loads((photo/'frames.json').read_bytes())['frames']
print('EARLIER_UNPAIRED_PHOTO',photo/frames[-1]['image'])
print('PAIRING_WARNING','No motor response during this earlier camera capture; not paired with04:34 encoders.')
