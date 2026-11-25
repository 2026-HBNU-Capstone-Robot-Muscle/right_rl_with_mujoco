"""Compare historical photo estimates and saved model phases, without devices."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT
from research_rotation_body_hold_20261104 import angle
p=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3';z=np.load(p/'initial_state.npz');t=np.load(p/'trajectory.npz');m=mujoco.MjModel.from_binary_path(str(p/'model.mjb'));d=mujoco.MjData(m);body=m.body('cube').id
rotations=[]
for target in (1,646,1417):
    index=int(np.argmin(abs(t['samples']-target)));d.qpos[:]=t['qpos'][index];mujoco.mj_forward(m,d);rotations.append(d.xmat[body].reshape(3,3).copy())
B0=z['initial_cube_rotation'];up=z['up'];model=dict(placement_to_grasp_deg=angle(rotations[1],B0,up),grasp_to_turn_end_deg=angle(rotations[2],rotations[1],up),placement_to_turn_end_deg=angle(rotations[2],B0,up),nearest_samples=[int(t['samples'][int(np.argmin(abs(t['samples']-x)))]) for x in (1,646,1417)])
photopath=ROOT/'model_discrepancy/saved_marker_rotation/review.json';photos=json.loads(photopath.read_bytes());second=photos['runs'][-1];accepted=[v for v in second['comparisons'] if v['frame']=='camera' and v['accepted']]
report=dict(status='phase_correspondence_diagnostic_only',saved_model_estimates=model,saved_photo_estimates=accepted,photographic_marker_dimensions_and_mounting_uncalibrated=True,camera_motion_not_independently_verified=True,physical_exact90_verified=False,devices_opened=False)
(ROOT/'model_discrepancy/phase_yaw_comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
