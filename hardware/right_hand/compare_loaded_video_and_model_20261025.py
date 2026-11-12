import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/actual_loaded_review_171607'
t=np.load(ROOT/'right_hand_rotation_20261024/completed_empty_path_review_20261025/measured_nominal_cube_replay_kinematic_v2.npz');q=t['qpos'][:,-4:];q=q/np.linalg.norm(q,axis=1,keepdims=True)
angles=np.degrees(2*np.arccos(np.clip(abs(q@q[0]),0,1)));times=t['times'];rows=[]
for s in (0,5,10,15,20,25,30,35,38,42):
    i=int(np.argmin(abs(times-s)));rows.append(dict(t=float(times[i]),conditional_empty_based_model_rotation_deg=float(angles[i])))
ref=json.loads((ROOT/'right_hand_rotation_20261024/original_sequence_clearance/reverse_original_native_verification.json').read_bytes())
initial=next(r for r in ref['runs'] if r.get('mode')=='moving_path' and r['gain_scale']==1 and r['friction']==.8 and r['offset_from_nominal_mm']==[0,0])['initial_cube_qpos']
print('OLD_MODEL',rows);print('INITIAL_CUBE_QPOS',initial)
(OUT/'conditional_model_timeline_comparison.json').write_text(json.dumps(dict(model_rows=rows,source_initial_cube_qpos=initial,
    observation_source='loaded_video_timeline.json',initial_cube_pose_in_hand_not_measured=True,model_initial_cube_pose_not_registered_to_actual_trial=True,
    model_prediction_mismatch_requires_review=True,loaded_response_is_measured_but_contact_model_not_validated=True),indent=2),encoding='utf-8')
