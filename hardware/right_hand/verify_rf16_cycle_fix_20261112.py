"""Recorded-state software comparison is not a prediction of a new real run."""
import json
import numpy as np
from goal_aware_progress_guard_20261025 import GoalAwareProgress,GoalAwareProgressFault,metadata
from nominal_cycle_plan_20261112 import ROOT,SOURCE,OUT,candidate
z=candidate(); raw=json.loads((ROOT/'nominal_two_turn_trial_20261112_055048_322766/result.json').read_bytes())
samples=[s for s in raw['samples'] if s['phase']=='stage_13_cycle_first_rotation']
faults=[]
for cap in (None,2177):
    guard=GoalAwareProgress(); fault=None
    for s in samples:
        goal=np.asarray(s['goal']).copy()
        if cap is not None: goal[15]=min(goal[15],cap)
        try: guard.check(s['phase'],[15],goal,np.asarray(s['actual']),s['t'])
        except GoalAwareProgressFault as exc: fault=exc.details;break
    faults.append(fault)
assert faults[0] and faults[0]['motor_id']==16 and faults[1] is None
original=np.load(ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/initial_second_turn_hold_replay/research_goals.npz',allow_pickle=False)
assert np.array_equal(z['goals'][:,:15],original['goals'][:,:15])
assert np.all(z['goals'][:,15]<=original['goals'][:,15])
assert np.max(abs(np.diff(np.vstack([z['start'],z['goals']]),axis=0)))<=2
r={'passed':True,'other15_goal_columns_unchanged':True,'only_ID16_goal_reduced':True,'goal_step_max2_preserved':True,
   'original_recorded_trace_ID16_fault_reproduced':faults[0],'capped_goal_on_same_recorded_actuals_ID16_fault':faults[1],
   'actual_new_robot_response_not_predicted_or_tested':True,'progress_guard_policy_unchanged':metadata(),
   'real_devices_opened':0,'assistant_motor_writes':0}
(OUT/'rf16_2177_v2/software_verification.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:r[k] for k in ('passed','other15_goal_columns_unchanged','only_ID16_goal_reduced','actual_new_robot_response_not_predicted_or_tested')}))
