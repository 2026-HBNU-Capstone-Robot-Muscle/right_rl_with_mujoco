"""Full placement -> observed forward -> proposed body grip, no state reset."""
import os
os.environ['JAX_PLATFORMS']='cpu'
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from sac_cube_model import set_cube_physics
from cube_contact_material_policy_20261029 import apply_material
from research_rotation_body_hold_20261104 import inspect_for
from replay_body_hold_from_placement_20261104 import initial
from canonical_grip_dynamics_20261105 import run
from dynamic_common_grip_search_20261105 import CP,ROOT,R

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--front',type=float,choices=(-15.,-10.,-5.),required=True);ap.add_argument('--nominal-only',action='store_true');ap.add_argument('--no-certify',action='store_true');ap.add_argument('--gain',type=float,choices=(.5,1.,1.5));ap.add_argument('--friction',type=float,choices=(.2,.3,.5));ap.add_argument('--candidate',type=Path);ap.add_argument('--output-tag',default='full_placement');a=ap.parse_args()
    candidate=a.candidate or ROOT/'recentered_free/i-77_m+32_t-48/from_reference/candidate.npz';out=ROOT/f'{a.output_tag}/front{a.front:g}';out.mkdir(parents=True,exist_ok=True);path=np.load(candidate);old=np.load(R/'right_hand_rotation_20261104/forward_hold/candidate.npz');forward=len(old['goals']);assert np.array_equal(path['start'],old['start']) and np.array_equal(path['goals'][:forward],old['goals']);assert np.max(np.abs(np.diff(np.vstack([path['start'],path['goals']]),axis=0)))<=2
    prior=json.loads((R/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());up=np.load(CP/'end_state.npz')['up'];cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,out/'scene.xml',book_projection_mm=a.front);correct_model(m,np.array(prior['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=.001;m.opt.gravity[:]=-9.81*up;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);hand,_,_=inspect_for(m);gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();cases=[]
    for scale in ((a.gain,) if a.gain is not None else (1.,) if a.nominal_only else (1.,.5,1.5)):
      for friction in ((a.friction,) if a.friction is not None else (.3,) if a.nominal_only else (.3,.2,.5)):
        began=time.perf_counter();m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;material=apply_material(m,friction,hand,m.geom('cube').id);seed=next(x for x in prior['loaded_physics'] if (x['book_front_mm'],x['gain_scale'],x['friction'])==(a.front,scale,friction));z,placement=initial(m,path['start'],up,np.array(seed['settled_cube_qpos']));caseout=out/f'g{scale:g}_f{friction:g}';caseout.mkdir(exist_ok=True);np.savez_compressed(caseout/'initial_state.npz',**z)
        row=dict(book_front_mm=a.front,gain_scale=scale,friction=friction,initial_placement=placement,material=material)
        if placement['passed']:
            row['continuous']=run(m,z,path['goals'],forward,caseout/'end_state.npz',caseout/'trajectory.npz' if (scale,friction)==(1.,.3) else None,not a.no_certify)
            if (scale,friction)==(1.,.3):mujoco.mj_saveModel(m,str(caseout/'model.mjb'))
        else:row['continuous']=None
        row['elapsed_seconds']=time.perf_counter()-began;cases.append(row);report=dict(status='continuous_placement_forward_common_grip_research',candidate=str(candidate.relative_to(R)),candidate_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),forward_source_unchanged=True,goals=len(path['goals']),nominal_seconds=len(path['goals'])*.075,cases=cases,all_initial_placements_passed=all(c['initial_placement']['passed'] for c in cases),all_safety_passed=all(c['continuous'] and c['continuous']['safety_passed'] for c in cases),all_tasks_passed=all(c['continuous'] and c['continuous']['whole_grip_task_passed'] for c in cases),cube_pose_and_velocity_never_reset_after_placement=True,native_contacts_force_and_retention_guards_unchanged=True,hardware_execution_allowed=False,physical_model_registration_verified=False,physical_grasp_verified=False,devices_opened=False)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');short=None if row['continuous'] is None else {k:v for k,v in row['continuous'].items() if k not in ('trace','final_rotation_world','final_contact_N')};print(json.dumps({k:v for k,v in row.items() if k not in ('continuous','material')}|{'continuous':short}),flush=True)
if __name__=='__main__':main()
