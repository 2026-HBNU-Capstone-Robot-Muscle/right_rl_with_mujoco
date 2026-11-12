"""Test load resistance of a temporary low grip without claiming90 completion.

The original83.58deg candidate is intentionally not renamed a90deg path.
All hand goals stay fixed and the unchanged contact/range/clearance filters
apply throughout each10.05s trial. Loads are uncalibrated hypotheses.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import hashlib,json,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from evaluate_canonical_grip_reaction_20261105 import evaluate

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    source=ROOT/'countergrip_level/seed0_source1_lift0_normal2.5_n900_resume_lower_staged_freebase/end_state.npz';model=ROOT/'torque_cap_sensitivity/cap0.15/model.mjb';z=dict(np.load(source));z['required_lower_contacts']=np.asarray(['if_tip','th_tip']);m=mujoco.MjModel.from_binary_path(str(model));out=ROOT/'temporary_handoff_load';out.mkdir(exist_ok=True);cases=[]
    for torque in (0.,-.003,.003,-.01,.01):
        began=time.perf_counter();v=evaluate(m,z,torque,out/f'torque{torque:+g}.npz');cases.append(v);report=dict(status='temporary_lower_grip_load_filter_only',source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_model_sha256=sha(model),cases=cases,source_yaw_deg=83.5798694166625,source_up_tilt_deg=5.69465,source_failed_original90_and_upright_target=True,passing_load_tests_cannot_override_pose_failure=True,temporal_clearance_required_only_for_left_approach_turn_retreat=True,physical_left_load_not_calibrated=True,devices_opened=False,hardware_execution_allowed=False);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(torque=torque,passed=v['load_filter_passed'],yaw_drift=v['max_yaw_change_deg'],tilt_drift=v['max_tilt_change_deg'],shift_mm=v['max_center_shift_mm'],gap_mm=v['minimum_separating_gap_1ms_mm'],contact_fraction=v['opposed_lower_body_contact_fraction'],reasons=v['reasons'],seconds=time.perf_counter()-began)),flush=True)
if __name__=='__main__':main()
