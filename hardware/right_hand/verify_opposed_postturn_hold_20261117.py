"""Check whether the reached, translated small-turn pose can hold unaided.

The parent fixed-station task remains failed. This separate diagnostic measures
hold drift from the actual reached pose, without moving or resetting the cube.
"""
from pathlib import Path
import sys,json,time
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from audit_opposed_working_grip_20261117 import MODEL,replay,sha
from research_rotation_body_hold_20261104 import motor

parent=ROOT/'right_hand_rotation_20261117/opposed_lower_grip/next_turn5p0_palmclear_margin8_fingerclear_allpalms'
prior=json.loads((parent/'review.json').read_bytes())
if prior['stop']['reason']!='bounded_turn_not_reached' or prior['goals']!=400:
    raise RuntimeError('Expected completed safe fixed-station diagnostic')
source=parent/'end_state.npz'
out=parent/'reached_pose_fixed_hold30'
out.mkdir(exist_ok=False)
z=dict(np.load(source,allow_pickle=False))
goal=motor(z['ctrl'])
m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
began=time.perf_counter()
result=replay(m,z,np.tile(goal,(400,1)),0,out)
result.update(parent_fixed_station_task_passed=False,parent_turn_deg=prior['actual_turn_deg'],
    parent_turn_target_error_deg=prior['target_error_deg'],parent_translation_mm=prior['center_error_mm'],
    parent_review_sha256=sha(parent/'review.json'),source_sha256=sha(source),model_sha256=sha(MODEL),
    script_sha256=sha(Path(__file__)),new_SAC_training=False,elapsed_seconds=time.perf_counter()-began)
(out/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('timeline','certificate')},ensure_ascii=True),flush=True)
