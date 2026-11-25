"""Full free-cube placement sensitivity at deliberately wider initial angles.

An initial yaw is applied only before the normal gravity seating procedure.
The reported turn is relative to the settled initial cube, never a changed
angle label. No endpoint cube reset, weld or device access is used.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,time,hashlib
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R
from replay_body_hold_from_placement_20261104 import initial
from canonical_grip_dynamics_20261105 import run
from canonical_grip_task_review_20261105 import task_reasons

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--yaw',type=int,choices=(-30,-20,-10,10,20,30),required=True);a=ap.parse_args()
    folder=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3'
    candidate=ROOT/'tip_squeeze/id1/plus12/candidate.npz';path=np.load(candidate)
    source=folder/'model.mjb';m=mujoco.MjModel.from_binary_path(str(source));up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    prior=json.loads((R/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes())
    seed=np.array(next(c for c in prior['loaded_physics'] if (c['book_front_mm'],c['gain_scale'],c['friction'])==(-10.,1.,.3))['settled_cube_qpos'])
    B=Rotation.from_quat(np.r_[seed[4:],seed[3]]).as_matrix();Br=Rotation.from_rotvec(up*np.radians(a.yaw)).as_matrix()@B
    q=Rotation.from_matrix(Br).as_quat();seed[3:]=np.r_[q[3],q[:3]]
    out=ROOT/f'wide_initial_yaw/yaw{a.yaw:+d}';out.mkdir(parents=True,exist_ok=True);began=time.perf_counter()
    z,placement=initial(m,path['start'],up,seed);np.savez_compressed(out/'initial_state.npz',**z)
    dynamic=run(m,z,path['goals'],1417,out/'end_state.npz',out/'trajectory.npz',True) if placement['passed'] else None
    reasons=task_reasons(dynamic)
    if not placement['passed']:reasons.append('initial_placement')
    target_reached=bool(dynamic and dynamic['safety_passed'] and abs(dynamic['final_yaw_from_placement_deg']-90)<5 and not reasons)
    report=dict(status='wide_initial_yaw_full_free_cube_replay',initial_yaw_hypothesis_deg=a.yaw,candidate=str(candidate.relative_to(R)),candidate_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),source_model=str(source.relative_to(R)),source_model_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),placement=placement,dynamic=dynamic,common_grip_task_reasons=reasons,quarter_turn90_and_common_filter_passed=target_reached,all_resets_only_before_gravity_seating=True,no_cube_pose_reset_after_placement=True,initial_yaw_is_not_a_certified_physical_placement_instruction=True,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(initial_yaw=a.yaw,placement=placement['passed'],safety=None if not dynamic else dynamic['safety_passed'],whole_yaw=None if not dynamic else dynamic['final_yaw_from_placement_deg'],tilt=None if not dynamic else dynamic['final_up_tilt_deg'],task_reasons=reasons,quarter90=target_reached,elapsed=report['elapsed_seconds'])),flush=True)
if __name__=='__main__':main()
