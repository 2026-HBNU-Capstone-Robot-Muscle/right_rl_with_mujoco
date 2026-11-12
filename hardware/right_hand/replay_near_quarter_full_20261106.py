"""Replay an initial placement through the complete recorded near-quarter path."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import hashlib,json,time
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R
from canonical_grip_dynamics_20261105 import run
from checkpoint_quarter_review_20261106 import quarter_reasons
from cube_task_frame_20261105 import orientation_error_deg

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    case=ROOT/'quasistatic_wrench_path/quarter_cap0.15_native_cone_potential_contacts_hybrid_motion_lift4_phase5_margin8';source=json.loads((case/'review.json').read_bytes());folder=(R/source['source_state']).parent;model=folder/'model.mjb';assert sha(model)==source['source_model_sha256']
    prefix=ROOT/'tip_squeeze/id1/plus12/candidate.npz';p=np.load(prefix);suffix=np.load(case/'suffix.npz');assert np.array_equal(p['goals'][-1],suffix['start'])
    rows=np.vstack([p['goals'],suffix['goals'],np.tile(suffix['goals'][-1],(134,1))]);maxstep=int(np.max(abs(np.diff(np.vstack([p['start'],rows]),axis=0))));assert maxstep<=2
    out=ROOT/'near_quarter_full_replay';out.mkdir(exist_ok=True);np.savez_compressed(out/'candidate.npz',start=p['start'],goals=rows)
    m=mujoco.MjModel.from_binary_path(str(model));z=np.load(folder/'initial_state.npz');began=time.perf_counter();v=run(m,z,rows,len(rows)-134,out/'end_state.npz',out/'trajectory.npz',True);common=np.load(folder/'end_state.npz')['qpos'][-7:-4];target=Rotation.from_rotvec(z['up']*np.pi/2).as_matrix()@z['initial_cube_rotation'];why=quarter_reasons(v,common,True,orientation_error_deg(v['final_rotation_world'],target))
    report=dict(status='full_initial_placement_near_quarter_recorded_goal_replay',source_model=str(model.relative_to(R)),source_model_sha256=sha(model),source_initial_state=str((folder/'initial_state.npz').relative_to(R)),source_initial_state_sha256=sha(folder/'initial_state.npz'),retained_prefix_sha256=sha(prefix),source_controller_review=str((case/'review.json').relative_to(R)),source_controller_review_sha256=sha(case/'review.json'),source_suffix_sha256=sha(case/'suffix.npz'),candidate_sha256=sha(out/'candidate.npz'),maximum_goal_increment_ticks=maxstep,retained_prefix_goals=len(p['goals']),additional_motion_goals=len(suffix['goals']),fixed_hold_goals=134,total_goals=len(rows),dynamic=v,quarter_and_common_grip_passed=not why,quarter_and_common_grip_reasons=why,free_cube_never_reset_during_combined_path=True,perfect_simulated_feedback_was_used_only_to_generate_saved_suffix=True,not_a_physical_policy_or_execution=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(completed=v['completed_goals'],total=len(rows),safety=v['safety_passed'],yaw=v['final_yaw_from_placement_deg'],tilt=v['final_up_tilt_deg'],contact=v['final_contact_N'],quarter_common=not why,reasons=why)),flush=True)
if __name__=='__main__':main()
