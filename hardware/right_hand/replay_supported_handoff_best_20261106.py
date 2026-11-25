"""Replay the optimized supported grip with every original1ms guard."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import hashlib,json
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from canonical_grip_dynamics_20261105 import run
from research_rotation_body_hold_20261104 import restore
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from checkpoint_quarter_review_20261106 import quarter_reasons
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    proposal=ROOT/'supported_handoff_search_mf_local/review.json';p=json.loads(proposal.read_bytes());source=R/p['source_state'];model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb';assert sha(source)==p['source_state_sha256'] and sha(model)==p['source_model_sha256'];path=proposal.parent/'best_candidate.npz';rows=np.load(path)['goals'];z=np.load(source);m=mujoco.MjModel.from_binary_path(str(model));out=ROOT/'supported_handoff_first_full';out.mkdir(exist_ok=True);v=run(m,z,rows,1,out/'end_state.npz',out/'trajectory.npz',True,5,('if_tip','th_tip'));end=restore(m,np.load(out/'end_state.npz'));target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);constant=bool(np.array_equal(rows[-134:],np.tile(rows[-1],(134,1))));why=quarter_reasons(v,p['best_proposal']['nominal_terminal_center_world_m'],constant,orientation_error_deg(end.xmat[m.body('cube').id].reshape(3,3),target));report=dict(status='optimized_supported_grip_full1ms_guard_replay',source_proposal=proposal.relative_to(R).as_posix(),source_proposal_sha256=sha(proposal),source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_goals=path.relative_to(R).as_posix(),source_goals_sha256=sha(path),source_model_sha256=sha(model),dynamic=v,checkpoint90_and_grip_passed=not why,reasons=why,full_initial_placement_not_replayed=True,devices_opened=False,hardware_execution_allowed=False);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(safe=v['safety_passed'],yaw=v['final_yaw_from_placement_deg'],tilt=v['final_up_tilt_deg'],gap=v['settled_tail']['minimum_separating_plane_gap_1ms_mm'],fraction=v['settled_tail']['opposed_lower_body_contact_fraction'],reasons=why,worst=v['worst'])),flush=True)
if __name__=='__main__':main()
