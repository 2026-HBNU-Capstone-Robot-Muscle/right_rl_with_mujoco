"""Compare nearby initial placements using the unchanged historical goal path.

Placement differs only before seating. Loaded replay never resets the cube or
adds forces. These offsets are model hypotheses, not physical placement advice.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from replay_body_hold_from_placement_20261104 import initial
from canonical_grip_dynamics_20261105 import run
from canonical_grip_task_review_20261105 import task_reasons

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--full-quarter',action='store_true');args=ap.parse_args()
    folder=ROOT/'torque_cap_sensitivity/cap0.15';model=folder/'model.mjb';baseline=json.loads((folder/'review.json').read_bytes());candidate=ROOT/'tip_squeeze/id1/plus12/candidate.npz';assert baseline['candidate_sha256']==sha(candidate)
    if args.full_quarter:
        candidate=ROOT/'near_quarter_full_replay/candidate.npz';near=json.loads((candidate.parent/'review.json').read_bytes());baseline['dynamic']=near['dynamic']
    path=np.load(candidate);m=mujoco.MjModel.from_binary_path(str(model));up=-m.opt.gravity/np.linalg.norm(m.opt.gravity);prior=json.loads((R/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());seed=np.array(next(c for c in prior['loaded_physics'] if (c['book_front_mm'],c['gain_scale'],c['friction'])==(-10.,1.,.3))['settled_cube_qpos']);out=ROOT/('initial_station_full_quarter_replay' if args.full_quarter else 'initial_station_replay');out.mkdir(exist_ok=True);results=[]
    for x_mm in (-12,0,12):
      for y_mm in (0,8,16):
        case=out/f'x{x_mm:+d}_y{y_mm:+d}';case.mkdir(exist_ok=True);began=time.perf_counter();placement=None;v=None;failure=None
        if x_mm==0 and y_mm==0:placement=baseline['placement'];v=baseline['dynamic']
        else:
            modified=seed.copy();modified[:3]+=np.array([x_mm,y_mm,0.])/1000
            try:
                z,placement=initial(m,path['start'],up,modified);np.savez_compressed(case/'initial_state.npz',**z)
                if placement['passed']:v=run(m,z,path['goals'],1417,case/'end_state.npz',None,True)
            except (ValueError,RuntimeError) as exc:failure=str(exc)
        why=task_reasons(v)
        if failure or not placement or not placement['passed']:why.append('initial_placement')
        if v is None or abs((v['final_yaw_from_placement_deg']-90+180)%360-180)>=5:why.append('whole_cube_not90')
        row=dict(initial_hand_X_offset_mm=x_mm,initial_hand_Y_offset_mm=y_mm,placement=placement,placement_error=failure,dynamic=v,quarter_and_final_grip_passed=not why,reasons=why,elapsed_seconds=time.perf_counter()-began,baseline_reused=(x_mm==0 and y_mm==0));results.append(row);report=dict(status='unchanged_recorded_motion_with_initial_station_hypotheses',candidate=candidate.relative_to(R).as_posix(),candidate_sha256=sha(candidate),source_model_sha256=sha(model),cases=results,initial_orientation_not_offset_to_fake_rotation=True,all_pose_adjustments_only_before_seating=True,initial_reference_measured_after_seating=True,cube_never_reset_or_forced_during_loaded_path=True,physical_initial_placement_coordinates_not_calibrated=True,native_geometry_force_caps_and_dynamic_guards_unchanged=True,devices_opened=False,hardware_execution_allowed=False);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(offsets=[x_mm,y_mm],placement=None if placement is None else placement['passed'],safety=None if v is None else v['safety_passed'],yaw=None if v is None else v['final_yaw_from_placement_deg'],grip=None if v is None else v['whole_grip_task_passed'],quarter_grip=not why,reasons=why)),flush=True)
if __name__=='__main__':main()
