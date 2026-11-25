"""Test middle-finger retreat after the full near-quarter replay, offline only.

The cube currently rests on palm/index surfaces. This checks whether the middle
finger can clear the upper layer without moving that cube; it does not silently
replace the existing MF+TH grip acceptance test with weaker contact names.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import hashlib,json
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R,ramp
from research_rotation_body_hold_20261104 import motor
from canonical_grip_dynamics_20261105 import run

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    source=ROOT/'near_quarter_full_replay';r=json.loads((source/'review.json').read_bytes());model=R/r['source_model'];assert sha(model)==r['source_model_sha256'];m=mujoco.MjModel.from_binary_path(str(model));z=np.load(source/'end_state.npz');start=motor(z['ctrl']);preferred=np.load(ROOT/'torque_cap_sensitivity/cap0.15/end_state.npz');goal=start.copy();goal[8:12]=motor(preferred['ctrl'])[8:12]
    out=ROOT/'postturn_middle_clearance';out.mkdir(exist_ok=True);rows=ramp(start,goal,134);np.savez_compressed(out/'candidate.npz',start=start,goals=rows)
    v=run(m,z,rows,1,out/'end_state.npz',out/'trajectory.npz',True)
    report=dict(status='free_cube_middle_retreat_after_near_quarter',source_review=str((source/'review.json').relative_to(R)),source_review_sha256=sha(source/'review.json'),source_state_sha256=sha(source/'end_state.npz'),source_model_sha256=sha(model),candidate_sha256=sha(out/'candidate.npz'),only_middle_goals_change=True,geometry_force_caps_native_limits_and_dynamic_guards_unchanged=True,dynamic=v,new_lower_grip_not_certified=True,hardware_execution_allowed=False,devices_opened=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(safety=v['safety_passed'],yaw=v['final_yaw_from_placement_deg'],shift=v['regrasp_translation_mm'],upper_gap=v['settled_tail']['minimum_separating_plane_gap_1ms_mm'],contacts=v['final_contact_N'],prior_grip_filter=v['whole_grip_task_passed'])),flush=True)
if __name__=='__main__':main()
