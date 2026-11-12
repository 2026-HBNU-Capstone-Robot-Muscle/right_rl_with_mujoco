"""Compact saved-goal inspection, no device access."""
import json
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip'
old=np.load(R/'right_hand_rotation_20261104/forward_hold/candidate.npz');first=int(np.flatnonzero(old['phases']=='stage_7_horizontal_yaw')[0]);current=np.load(O/'tip_squeeze/id1/plus12/candidate.npz')['goals'][-1]
print(json.dumps(dict(pre_yaw=old['goals'][first-1].tolist(),post_yaw=old['goals'][-1].tolist(),new_common=current.tolist(),middle_motor_order=[12,10,11,9],middle_common_ticks=current[np.array([12,10,11,9])-1].tolist()),ensure_ascii=False))
