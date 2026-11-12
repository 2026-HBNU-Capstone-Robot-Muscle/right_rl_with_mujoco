"""Document the reused residual policy's proposal authority, without claiming infeasibility."""
from pathlib import Path
import json
import hashlib
import numpy as np

OUT=Path(__file__).resolve().parent
F=OUT/'next_cycle/handoff_refinement_20261110'
SIM_IDS=np.array([8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1])
TICK=2*np.pi/4096


def main():
    settings=[]
    for name in ('sac_ccd50_repair_10k','sac_ccd50_recovery_long_10k','sac_ccd50_early_recovery_10k'):
        path=OUT/name/'settings.json';r=json.loads(path.read_text())
        assert r['residual_rad']==.01 and r['max_goal_ticks']==2 and r['control_seconds']==.075
        settings.append(dict(run=name,source_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    first=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation/end_state.npz'
    endpoint=F/'endpoint_screen/qualified3/end_state.npz'
    start=np.load(first,allow_pickle=False)['ctrl'];end=np.load(endpoint,allow_pickle=False)['ctrl']
    delta=np.empty(16);delta[SIM_IDS-1]=(end-start)/TICK
    result=dict(status='residual_action_authority_audit',runs=settings,
        residual_rad=.01,residual_degrees=float(np.degrees(.01)),residual_ticks_before_rounding=float(.01/TICK),
        proposal_rounding_max_extra_ticks=.5,executed_per_cycle_limit_ticks=2,cycle_seconds=.075,
        proposal_is_tied_to_old_reference_not_incremental_regrasp=True,
        phase_advances_with_each_acknowledged_goal_in_current_research_controller=True,
        initialized_ring_endpoint_command_difference_ticks_from_old_first_hold=delta.tolist(),
        ring_tip_to_base_ids=[13,15,14,16],ring_tip_to_base_delta_ticks=delta[np.array([13,15,14,16])-1].tolist(),
        initialized_endpoint_is_one_candidate_not_a_minimum_required_motion=True,
        rate_governor_can_leave_executed_goal_temporarily_outside_proposal_tube=True,
        does_not_prove_all_existing_policies_useless_or_new_training_mandatory=True,
        devices_opened=0,hardware_execution_allowed=False)
    (F/'action_authority_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(residual_degrees=result['residual_degrees'],residual_ticks=result['residual_ticks_before_rounding'],
                         ring_delta_ticks=result['ring_tip_to_base_delta_ticks'])),flush=True)


if __name__=='__main__':main()
