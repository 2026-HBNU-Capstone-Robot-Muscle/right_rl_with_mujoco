"""Free-cube transitions to static workspace proposals; no pinned dynamics."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,hashlib,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R,CP,ramp
from research_rotation_body_hold_20261104 import motor
from canonical_grip_dynamics_20261105 import run
from canonical_grip_task_review_20261105 import task_reasons

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--origin',choices=('forward','common'),required=True);ap.add_argument('--endpoint',choices=('xpos','zneg'),default='xpos');a=ap.parse_args();endpoint=ROOT/('sliding_workspace_grip/weight1_raised_xpos' if a.endpoint=='xpos' else 'sliding_workspace_grip/weight1_open_zneg');proposal=json.loads((endpoint/'review.json').read_bytes());assert proposal['proposal_accepted'];target=np.load(endpoint/'endpoint.npz');folder=CP if a.origin=='forward' else ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3';source=folder/'end_state.npz';m=mujoco.MjModel.from_binary_path(str(folder/('end_model.mjb' if a.origin=='forward' else 'model.mjb')));z=np.load(source);prefix_path=R/'right_hand_rotation_20261104/forward_hold/candidate.npz' if a.origin=='forward' else ROOT/'tip_squeeze/id1/plus12/candidate.npz';prefix=np.load(prefix_path);assert np.array_equal(prefix['goals'][-1],motor(z['ctrl']));out=ROOT/f'workspace_transition/{a.endpoint}_{a.origin}';out.mkdir(parents=True,exist_ok=True);cases=[]
    for preload in proposal['model_only_preload_proposals']:
        if not preload['native_goal_valid']:continue
        force=preload['assumed_each_contact_normal_force_N'];caseout=out/f'force{force:g}';caseout.mkdir(exist_ok=True);goal=np.array(preload['goal']);rows=ramp(motor(z['ctrl']),goal,134);began=time.perf_counter();d=run(m,z,rows,1,caseout/'end_state.npz',caseout/'trajectory.npz',True);why=task_reasons(d);row=dict(assumed_each_contact_preload_N=force,dynamic=d,task_reasons=why,grip_task_passed=not why,final_center_error_to_static_proposal_mm=float(np.linalg.norm(np.array(d['final_center_world_m'])-target['cube_center'])*1000),elapsed_seconds=time.perf_counter()-began);cases.append(row)
        np.savez_compressed(caseout/'candidate.npz',start=prefix['start'],goals=np.vstack([prefix['goals'],rows]),phases=np.r_[prefix['phases'],['workspace_body_regrasp']*(len(rows)-134),['workspace_body_hold']*134])
        report=dict(status='continuous_free_cube_workspace_grip_transition',origin=a.origin,endpoint=a.endpoint,source_state=str(source.relative_to(R)),source_state_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),proposal=str((endpoint/'review.json').relative_to(R)),proposal_sha256=hashlib.sha256((endpoint/'review.json').read_bytes()).hexdigest(),cases=cases,full_initial_placement_not_replayed=True,no_cube_pose_or_velocity_reset_in_transition=True,force_preload_is_an_unmeasured_model_hypothesis=True,current_PID_actuator_and_collision_bounds_unchanged=True,hardware_execution_allowed=False,devices_opened=False)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(origin=a.origin,endpoint=a.endpoint,preload=force,passed=not why,reasons=why,safety=d['safety_passed'],yaw=d['final_yaw_from_placement_deg'],regrasp_yaw=d['regrasp_yaw_change_deg'],tilt=d['final_up_tilt_deg'],gap=d['settled_tail']['minimum_separating_plane_gap_1ms_mm'],center_error=row['final_center_error_to_static_proposal_mm'],elapsed_seconds=row['elapsed_seconds'])),flush=True)
if __name__=='__main__':main()
