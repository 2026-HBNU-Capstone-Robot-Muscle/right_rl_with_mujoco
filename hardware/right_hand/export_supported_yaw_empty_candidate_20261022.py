"""Freeze an EMPTY rehearsal suffix; does not authorize a loaded trial."""
import hashlib
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261022'
source=OUT/'supported_yaw_result.json';screen=OUT/'supported_yaw_empty_suffix_screen.json'
r=json.loads(source.read_bytes());s=json.loads(screen.read_bytes())
assert s['passed'] and s['hardware_commands_sent']==0
base=np.asarray(r['base_goal_ticks']);q=np.asarray(r['candidate_goal_ticks'])
assert q.shape==(256,16) and np.max(abs(np.diff(np.vstack([base,q]),axis=0)))<=2
assert np.max(abs(q-base))<=256 and np.all(q[:,[12,14,15]]<=base[[12,14,15]])
data=dict(version='supported_yaw_empty_rehearsal_v1',cube_allowed=False,live_sac=False,
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    empty_suffix_screen_sha256=hashlib.sha256(screen.read_bytes()).hexdigest(),
    base_goal_ticks=base.tolist(),goal_ticks=q.tolist(),sample_period_s=.05,
    maximum_suffix_travel_ticks=int(np.max(abs(q-base))),
    rotation_goal_not_verified=True,loaded_test_authorized=False,
    algorithm='offline_CEM_fixed_path_NOT_SAC',purpose='EMPTY ring-involving motion/clearance observation only')
path=OUT/'supported_yaw_empty_candidate.json'
path.write_text(json.dumps(data,indent=2),encoding='utf-8')
print(path.name,hashlib.sha256(path.read_bytes()).hexdigest())
