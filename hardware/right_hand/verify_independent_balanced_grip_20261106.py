"""Release and hold a proposed90deg grip, independently initialized.

This cannot demonstrate reaching that pose. It checks whether the proposed
static force solution survives a free10.05s hold with quantized motor goals.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import hashlib,json
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import motor,restore
from hand_motor_mapping import ticks_to_sim
from canonical_grip_dynamics_20261105 import run
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target
from checkpoint_quarter_review_20261106 import quarter_reasons
from evaluate_canonical_grip_reaction_20261105 import evaluate

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb';reference=ROOT/'near_quarter_full_replay/end_state.npz';proposal=ROOT/'lower_countergrip_station_force2_span12_equilibrium/seed4/static_proposal.npz';p=np.load(proposal);z=dict(np.load(reference));m=mujoco.MjModel.from_binary_path(str(model));z['qpos']=z['qpos'].copy();z['qpos'][:16]=p['actual_joint_proposal'];z['qpos'][-7:-4]=p['cube_center'];q=Rotation.from_matrix(p['cube_rotation']).as_quat();z['qpos'][-4:]=np.r_[q[3],q[:3]];z['qvel']=np.zeros_like(z['qvel']);z['qacc_warmstart']=np.zeros_like(z['qacc_warmstart']);goals=motor(p['force_balanced_control']);z['ctrl']=ticks_to_sim(goals);z['required_lower_contacts']=np.asarray(['if_tip','th_tip']);out=ROOT/'independent_balanced_grip';out.mkdir(exist_ok=True);np.savez_compressed(out/'independent_initial_state.npz',**z);rows=np.tile(goals,(134,1));v=run(m,z,rows,1,out/'hold_end.npz',out/'trajectory.npz',True,5,('if_tip','th_tip'));end=restore(m,np.load(out/'hold_end.npz'));target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);why=quarter_reasons(v,p['cube_center'],True,orientation_error_deg(end.xmat[m.body('cube').id].reshape(3,3),target));loads=[]
    if not why:
        steady=dict(np.load(out/'hold_end.npz'));steady['required_lower_contacts']=np.asarray(['if_tip','th_tip'])
        for torque in (0.,-.003,.003,-.01,.01):
            load=evaluate(m,steady,torque,out/f'load{torque:+g}.npz');loads.append(load);print(json.dumps(dict(torque=torque,passed=load['load_filter_passed'],reasons=load['reasons'])),flush=True)
    report=dict(status='independently_initialized90_grip_free_hold',source_model_sha256=sha(model),source_reference_state_sha256=sha(reference),source_proposal=proposal.relative_to(R).as_posix(),source_proposal_sha256=sha(proposal),independent_initial_state_sha256=sha(out/'independent_initial_state.npz'),dynamic=v,reasons=why,independent_hold90_passed=not why,load_cases=loads,not_a_reaching_or_reorientation_trajectory=True,no_cube_pose_reset_after_independent_initialization=True,devices_opened=False,hardware_execution_allowed=False);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(passed=not why,yaw=v['final_yaw_from_placement_deg'],tilt=v['final_up_tilt_deg'],gap=v['settled_tail']['minimum_separating_plane_gap_1ms_mm'],pressure_fraction=v['settled_tail']['opposed_lower_body_contact_fraction'],worst=v['worst'],reasons=why)),flush=True)
if __name__=='__main__':main()
