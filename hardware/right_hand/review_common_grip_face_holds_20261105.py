"""Six independently initialized static face-label hold hypotheses, not flips."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json,hashlib
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT
from research_rotation_body_hold_20261104 import restore,motor
from cube_orientation_routes_20261028 import validate,NORMALS,NAMES
from canonical_grip_dynamics_20261105 import run
from canonical_grip_task_review_20261105 import task_reasons

def main():
    folder=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3';source=folder/'end_state.npz';m=mujoco.MjModel.from_binary_path(str(folder/'model.mjb'));saved=np.load(source);start=restore(m,saved);B0=start.xmat[m.body('cube').id].reshape(3,3).copy();seen={};cases=[];out=ROOT/'face_hold_hypotheses';out.mkdir(parents=True,exist_ok=True)
    for logical,(S,moves) in sorted(validate().items()):
        n=S.T@np.array([0,-1,0]);face=int(np.flatnonzero(np.all(NORMALS==n,axis=1))[0])
        if face in seen:continue
        seen[face]=True;z={k:saved[k].copy() for k in saved.files};q=Rotation.from_matrix(B0@S).as_quat();z['qpos'][-4:]=np.r_[q[3],q[:3]];z['qvel'][:]=0;z['qacc_warmstart'][:]=0;rows=np.tile(motor(z['ctrl']),(134,1));d=run(m,z,rows,1,out/f'face{face}_end.npz',None,True,top_face_id=face);why=task_reasons(d);cases.append(dict(top_face_id=face,top_face_name=NAMES[face],label_rotation=S.tolist(),task_passed=not why,reasons=why,dynamic=d));report=dict(status='six_static_label_initializations_then10second_holds',source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),cases=cases,independent_initial_pose_hypotheses_not_reorientation_trajectories=True,cube_geometry_is_an_equal_sided_rigid_box=True,gain_scale=1.,friction=.3,all_guards_and_actuator_properties_unchanged=True,physical_faces_not_executed_or_verified=True,hardware_execution_allowed=False,devices_opened=False)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(face=face,name=NAMES[face],passed=not why,reasons=why,tilt=d['final_up_tilt_deg'],gap=d['settled_tail']['minimum_separating_plane_gap_1ms_mm'])),flush=True)
if __name__=='__main__':main()
