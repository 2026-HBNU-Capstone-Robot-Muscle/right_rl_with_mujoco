"""Independently test a saved thumb/index lower grip; no hardware runner.

The candidate is initialized once for a hold test. It is not evidence that a
rotation or a transition from the current physical hand can reach this pose.
"""
from pathlib import Path
import hashlib,json,sys,time
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS,ticks_to_sim
from research_rotation_body_hold_20261104 import restore,motor
from canonical_grip_dynamics_20261105 import run


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out=ROOT/'right_hand_rotation_20261116/opposed_lower_grip'
    out.mkdir(exist_ok=False)
    prior=ROOT/'right_hand_rotation_20261105/canonical_body_grip'
    model=prior/'torque_cap_sensitivity/cap0.15/model.mjb'
    proposal=prior/'lower_countergrip_station_force2_span12_equilibrium/seed4/static_proposal.npz'
    source=prior/'countergrip_level/seed0_source1_lift0_normal2.5_n900_resume_lower_staged_freebase/end_state.npz'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()})
    z=dict(np.load(source,allow_pickle=False));p=np.load(proposal,allow_pickle=False)
    z['qpos']=z['qpos'].copy();z['qpos'][:16]=p['actual_joint_proposal']
    z['qpos'][-7:-4]=p['cube_center'];quat=Rotation.from_matrix(p['cube_rotation']).as_quat()
    z['qpos'][-4:]=np.r_[quat[3],quat[:3]]
    z['qvel']=np.zeros_like(z['qvel']);z['qacc_warmstart']=np.zeros_like(z['qacc_warmstart'])
    goal=motor(p['force_balanced_control']);z['ctrl']=ticks_to_sim(goal)
    z['initial_cube_rotation']=p['cube_rotation'].copy();z['initial_cube_center']=p['cube_center'].copy()
    np.savez_compressed(out/'independent_initial.npz',**z)
    np.savez_compressed(out/'posture_candidate.npz',goal_ticks=goal,
        sim_motor_ids=np.array(SIM_TO_MOTOR_IDS),cube_center=p['cube_center'],cube_rotation=p['cube_rotation'],
        hardware_execution_allowed=np.array(False))
    report=dict(status='independent_hold_running',intent=dict(front='Cube local +Z: thumb lower face',
        back='Cube local -Z: index lower face',middle='Optional later; not forced to contact',
        repeat_goal='rotation -> this common lower grip/hold -> next rotation -> same lower grip/hold'),
        model_sha256=sha(model),proposal_sha256=sha(proposal),source_sha256=sha(source),
        candidate_sha256=sha(out/'posture_candidate.npz'),goal_ticks=goal.tolist(),
        independent_initialized_pose=True,reaching_path_verified=False,continuous_cycle_verified=False,
        physical_grasp_verified=False,hardware_execution_allowed=False,devices_opened=False,
        research_torque_cap_nm=.15,torque_cap_is_not_450mA_conversion=True,
        friction_is_model_assumption=True,simulator_version=mujoco.__version__)
    path=out/'review.json';path.write_text(json.dumps(report,indent=2)+'\n')
    print('Testing independent TH/IF lower grip; no rotation or device access.',flush=True)
    began=time.perf_counter()
    result=run(m,z,np.tile(goal,(134,1)),1,out/'hold_end.npz',out/'hold_trace.npz',True,5,('if_tip','th_tip'))
    report.update(status='independent_hold_complete',hold=result,elapsed_seconds=time.perf_counter()-began)
    path.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(passed=result['whole_grip_task_passed'],safety=result['safety_passed'],
        completed=result['completed_goals'],stop=result['stop'],worst=result['worst'],
        tail=result['settled_tail'],report=str(path))),flush=True)


if __name__=='__main__':main()
