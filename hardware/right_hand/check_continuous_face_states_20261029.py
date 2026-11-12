"""Summarize face states and relative yaw from an uninterrupted physics rollout."""
import json
from pathlib import Path
import numpy as np,cv2
ROOT=Path(__file__).resolve().parent
path=ROOT/'right_hand_rotation_20261028/two_axis_connection/actual_handoff_yaw_diagnostic/whole_pitch_handoff_yaw_hypothesis/review.json'
r=json.loads(path.read_bytes());up=np.asarray(json.loads((ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1/seed.json').read_bytes())['up_world'])
def matrix(q):
    quat=np.asarray(q)[3:];angle=2*np.arctan2(np.linalg.norm(quat[1:]),quat[0]);length=np.linalg.norm(quat[1:])
    return cv2.Rodrigues(quat[1:]/length*angle)[0] if length>1e-12 else np.eye(3)
rows=[]
for case in r['cases']:
    phases=case['phase_checkpoints'];pitch=matrix(phases['stage_6_pitch_hold']['cube_qpos'])
    for phase,value in phases.items():
        R=matrix(value['cube_qpos']);rv=cv2.Rodrigues(R@pitch.T)[0].ravel();a=np.linalg.norm(rv)
        twist=np.degrees(2*np.arctan2((rv@up)/a*np.sin(a/2),np.cos(a/2))) if a>1e-9 else 0.
        rows.append(dict(phase=phase,sample=value['sample'],red_up_error_deg=value['red_up_error_deg'],green_up_error_deg=value['green_up_error_deg'],
            horizontal_twist_from_pitch_hold_deg=float(twist),cube_center_world_m=value['cube_qpos'][:3]))
out=dict(source=str(path),initial_pose_is_conditional=True,force_free_cube_trajectory_not_used=True,phase_summary=rows)
(path.parent/'face_states.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print(json.dumps(out))
