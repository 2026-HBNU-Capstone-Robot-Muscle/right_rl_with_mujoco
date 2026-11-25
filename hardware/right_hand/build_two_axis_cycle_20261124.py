"""Build a new compound candidate without changing either component."""
import json,numpy as np
from two_axis_cycle_plan_20261124 import ROOT,BASE,PERIOD,components,sha

def main():
    h,v,_=components();BASE.mkdir(parents=True,exist_ok=True)
    goals=np.vstack([h.goals,v.goals])
    phases=np.concatenate([np.array(['h_'+str(p) for p in h.phases]),
                           np.array(['v_'+str(p) for p in v.phases])])
    np.savez_compressed(BASE/'candidate.npz',start=h.start,goals=goals,phases=phases)
    sources=dict(h.manifest['source_hashes'])
    for name,digest in v.manifest['source_hashes'].items():
        if name in sources and sources[name]!=digest:raise RuntimeError('Component source disagreement: '+name)
        sources[name]=digest
    names=['two_axis_cycle_plan_20261124.py','build_two_axis_cycle_20261124.py',
        'leap_two_axis_cycle_20261124.py','leap_two_axis_cycle.cmd','test_two_axis_cycle_20261124.py',
        'right_hand_rotation_20261118/grip_rotation_return/operator_trial_manifest.json',
        'right_hand_rotation_20261118/grip_rotation_return/candidate.npz',
        'right_hand_rotation_20261118/vertical_direct_thumb_return/operator_trial_manifest.json',
        'right_hand_rotation_20261118/vertical_direct_thumb_return/candidate.npz']
    sources.update({name:sha(ROOT/name) for name in names})
    if any(sha(ROOT/name)!=digest for name,digest in sources.items()):raise RuntimeError('Two-axis source mismatch')
    review=dict(kind='exact_horizontal_G_vertical_G_concatenation',candidate_sha256=sha(BASE/'candidate.npz'),
        goals=len(goals),horizontal_goals=len(h.goals),vertical_goals=len(v.goals),
        horizontal_candidate_sha256=h.manifest['candidate_sha256'],
        vertical_candidate_sha256=v.manifest['candidate_sha256'],
        horizontal_G_hold_seconds=int(np.sum(h.phases=='return_fixed_grip_hold'))*PERIOD,
        source_goal_arrays_exact=True,source_endpoints_same_G=True,new_transition_waypoints=0,
        handoff_goal_step_ticks=int(np.max(abs(v.goals[0]-h.goals[-1]))),
        automatic_stage_transitions_enabled=True,maximum_ON_seconds=None,
        period_seconds=PERIOD,maximum_step_ticks=2,current_cap_ma=450,PID_unchanged=True,
        original_transport_code_used=True,horizontal_recorded_bounds_horizontal_only=True,
        vertical_original_bounds=True,cube_dynamics_evaluated=False,
        physical_two_axis_cycle_verified=False,tests_passed=0,source_hashes=sources)
    (BASE/'operator_trial_manifest.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(goals=len(goals),horizontal_goals=len(h.goals),vertical_goals=len(v.goals),
        nominal_core_seconds=len(goals)*PERIOD,handoff_goal_step_ticks=review['handoff_goal_step_ticks'],
        bound_sources=len(sources),devices_opened=False)))

if __name__=='__main__':main()
