"""Replay hardware held/progress policies on nominal model observations."""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'))
import mujoco
from native_cycle import assets,restore,Guard,motor,ticks_to_sim,save_json
from held_joint_guard import HeldJointGuard,HeldJointFault
from goal_aware_progress_guard_20261025 import GoalAwareProgress,GoalAwareProgressFault,metadata
from tip_first_trial_plan_20261112 import OUT,CANDIDATE,TipFirstTrialPlan,candidate,sha

def main():
    output=OUT/'runtime_guard_review.json'
    if output.exists():raise RuntimeError('Preserve guard replay')
    z=candidate();m,initial,_,_,provenance=assets('active_ccd50');d=restore(m,initial);g=Guard(m)
    p=object.__new__(TipFirstTrialPlan);held=None;progress=GoalAwareProgress();fault=None;observations=0;stop=[]
    for i,(phase,command) in enumerate(zip(z['phases'],z['goals'])):
        prior=motor(d.ctrl);before=motor(d.qpos[:16])
        if i==1417:held=HeldJointGuard(before,prior)
        if held is not None:held.advance(before,prior,command)
        d.ctrl[:]=ticks_to_sim(command)
        for _ in range(75):
            mujoco.mj_step(m,d);stop=g.check(d)
            if stop:break
        actual=motor(d.qpos[:16])
        if held is not None:
            try:
                held.check(actual,command,p.frozen_for_phase(str(phase)),str(phase))
                progress.check(str(phase),p.progress_for_phase(str(phase)),command,actual,(i+1)*.075)
                observations+=1
            except (HeldJointFault,GoalAwareProgressFault) as exc:fault=exc.details
        if stop or fault:break
    report=dict(passed=not stop and fault is None and i+1==len(z['goals']),done=i+1,total=len(z['goals']),
        new_phase_observations=observations,stop=stop,policy_fault=fault,candidate_sha256=sha(CANDIDATE),
        checked_plan_sha256=sha(ROOT/'tip_first_trial_plan_20261112.py'),checker_sha256=sha(Path(__file__)),
        first1417_uses_previously_executed_runtime=True,nominal_model_only=True,model_provenance=provenance,
        held_policy_version=HeldJointGuard.VERSION,progress_policy=metadata(),
        held_margin_ticks=20,policy_thresholds_changed=False,physical_response_predicted=False,devices_opened=0)
    save_json(output,report);print(json.dumps({k:v for k,v in report.items() if k not in ('model_provenance','progress_policy')}),flush=True)

if __name__=='__main__':main()
