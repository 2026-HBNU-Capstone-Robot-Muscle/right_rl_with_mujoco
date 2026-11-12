"""Keep the same optimized targets longer, then apply the unchanged90 filter.

No new motion targets, force, pose resets or tolerances are introduced. A
longer wait is only a model timing proposal, never a real execution command.
"""
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
    proposal=ROOT/'supported_handoff_search_mf_local/review.json';p=json.loads(proposal.read_bytes());source=R/p['source_state'];model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb';path=proposal.parent/'best_candidate.npz';raw=np.load(path);prefix=raw['goals'][:-134];hold=802;rows=np.vstack([prefix,np.tile(raw['goals'][-1],(hold,1))]);z=np.load(source);m=mujoco.MjModel.from_binary_path(str(model));out=ROOT/'supported_handoff_extended_settle';out.mkdir(exist_ok=True);np.savez_compressed(out/'candidate.npz',start=raw['start'],goals=rows);v=run(m,z,rows,1,out/'end_state.npz',out/'trajectory.npz',True,5,('if_tip','th_tip'));end=restore(m,np.load(out/'end_state.npz'));target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90);error=orientation_error_deg(end.xmat[m.body('cube').id].reshape(3,3),target);why=quarter_reasons(v,p['best_proposal']['nominal_terminal_center_world_m'],True,error);report=dict(status='unchanged_goal_extended_model_settle',source_proposal_sha256=sha(proposal),source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_model_sha256=sha(model),constant_goal_samples=hold,constant_goal_seconds=hold*.075,dynamic=v,orientation_error_deg=error,reasons=why,checkpoint90_and_grip_passed=not why,full_initial_placement_not_replayed=True,devices_opened=False,hardware_execution_allowed=False);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(safe=v['safety_passed'],yaw=v['final_yaw_from_placement_deg'],tilt=v['final_up_tilt_deg'],orientation_error=error,gap=v['settled_tail']['minimum_separating_plane_gap_1ms_mm'],fraction=v['settled_tail']['opposed_lower_body_contact_fraction'],passed=not why,reasons=why)),flush=True)
if __name__=='__main__':main()
