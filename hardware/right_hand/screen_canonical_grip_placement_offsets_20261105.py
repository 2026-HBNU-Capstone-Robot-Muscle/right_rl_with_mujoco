"""Initial placement sensitivity only: offsets are model hypotheses, not instructions."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import inspect_for
from cube_contact_material_policy_20261029 import apply_material
from replay_body_hold_from_placement_20261104 import initial
from canonical_grip_dynamics_20261105 import run
from canonical_grip_task_review_20261105 import task_reasons

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--forward-mm',type=int,choices=(-2,0,2),required=True);ap.add_argument('--candidate',type=Path,default=ROOT/'recentered_free/i-77_m-16_t-56/from_reference/candidate.npz');ap.add_argument('--matrix-tag',default='full_placement_v2');ap.add_argument('--output-tag',default='placement_offsets');a=ap.parse_args();out=ROOT/f'{a.output_tag}/forward{a.forward_mm:+d}';out.mkdir(parents=True,exist_ok=True);candidate=a.candidate;path=np.load(candidate);base=ROOT/a.matrix_tag/'front-10';m=mujoco.MjModel.from_binary_path(str(base/'g1_f0.3/model.mjb'));hand,_,_=inspect_for(m);apply_material(m,.3,hand,m.geom('cube').id);up=-m.opt.gravity/np.linalg.norm(m.opt.gravity);prior=json.loads((R/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());seed=np.array(next(c for c in prior['loaded_physics'] if (c['book_front_mm'],c['gain_scale'],c['friction'])==(-10.,1.,.3))['settled_cube_qpos']);B=Rotation.from_quat(np.r_[seed[4:],seed[3]]).as_matrix();forward=B[:,2]-up*(B[:,2]@up);forward/=np.linalg.norm(forward);side=np.cross(up,forward);cases=[];reference=json.loads((base/'review.json').read_bytes());nominal=next(c for c in reference['cases'] if (c['gain_scale'],c['friction'])==(1.,.3))
    if reference['candidate_sha256']!=hashlib.sha256(candidate.read_bytes()).hexdigest():raise ValueError('candidate/model source mismatch')
    for lateral in (-2,0,2):
      for yaw in (-5,0,5):
        began=time.perf_counter();caseout=out/f'side{lateral:+d}_yaw{yaw:+d}';caseout.mkdir(exist_ok=True)
        if (a.forward_mm,lateral,yaw)==(0,0,0):
            row=dict(forward_offset_mm=0,side_offset_mm=0,initial_yaw_offset_deg=0,initial_placement=nominal['initial_placement'],continuous=nominal['continuous'],reused_existing_nominal_evidence=str((base/'review.json').relative_to(R)))
        else:
            modified=seed.copy();modified[:3]+=(forward*a.forward_mm+side*lateral)/1000;rot=Rotation.from_rotvec(up*np.radians(yaw)).as_matrix()@B;q=Rotation.from_matrix(rot).as_quat();modified[3:]=np.r_[q[3],q[:3]];z,placement=initial(m,path['start'],up,modified);row=dict(forward_offset_mm=a.forward_mm,side_offset_mm=lateral,initial_yaw_offset_deg=yaw,initial_placement=placement,continuous=None)
            if placement['passed']:row['continuous']=run(m,z,path['goals'],1417,caseout/'end_state.npz',None,True)
        why=task_reasons(row['continuous'])
        if not row['initial_placement']['passed']:why.append('initial_placement')
        row.update(task_passed=not why,reasons=why,elapsed_seconds=time.perf_counter()-began);cases.append(row);report=dict(status='model_initial_placement_offset_sensitivity',candidate=str(candidate.relative_to(R)),candidate_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),gain_scale=1.,friction=.3,book_front_mm=-10.,cases=cases,offsets_are_uncalibrated_model_coordinates_only=True,all_cube_pose_resets_are_only_before_initial_placement=True,no_cube_pose_reset_during_loaded_motion=True,physical_placement_tolerance_not_verified=True,hardware_execution_allowed=False,devices_opened=False)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');d=row['continuous'];print(json.dumps(dict(offsets=[a.forward_mm,lateral,yaw],placement_passed=row['initial_placement']['passed'],task_passed=not why,reasons=why,yaw_change_deg=None if not d else d['regrasp_yaw_change_deg'],center=None if not d else d['final_center_world_m'],certificate_gap_mm=None if not d else d['settled_tail']['minimum_separating_plane_gap_1ms_mm'],elapsed_seconds=row['elapsed_seconds'])),flush=True)
if __name__=='__main__':main()
