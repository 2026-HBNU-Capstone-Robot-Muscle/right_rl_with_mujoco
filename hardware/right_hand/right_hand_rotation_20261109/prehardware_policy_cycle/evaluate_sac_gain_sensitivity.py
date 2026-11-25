"""Reuse the saved SAC actor with explicit model-only output-gain comparisons.

No training or actor file changes. Every gain retains the original native guard,
integer encoders, nearest legal targets and max2tick/75ms command increments.
"""
from pathlib import Path
import sys
import json
import numpy as np
from native_cycle import OUT,BOUND,save_json,sha
from evaluate_staged_shadow_model import main as evaluate,RepairAdmissibleController


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/sac_gain_sensitivity'
    if folder.exists():raise RuntimeError('Preserve prior sensitivity study')
    folder.mkdir();(folder/'source_evaluate_sac_gain_sensitivity.py').write_bytes(Path(__file__).read_bytes())
    summaries=[]
    for gain in (.01,.03,.05):
        tag='gain'+str(round(gain*1000)).zfill(3)+'_projection'
        def controller(bundle):
            c=RepairAdmissibleController(bundle)
            if c.meta['residual_rad']!=.01:raise RuntimeError('Unexpected original training scale')
            c.meta=dict(c.meta,residual_rad=gain)
            return c
        old_argv=sys.argv
        try:
            sys.argv=['evaluate_staged_shadow_model.py','--case','nominal','--model-kind','active_ccd50',
                '--integer-encoders','--admissible-correction','--hold-extra-steps','400',
                '--support-mode','whole_hand_v1','--tag-suffix',tag]
            evaluate(controller_factory=controller,research_metadata=dict(
                actor_scaling_research=dict(original_training_residual_rad=.01,applied_residual_rad=gain,
                    same_original_actor_weights=True,retrained_for_this_gain=False,
                    nearest_native_range_proposals_all_phases=True,
                    source_sha256=sha(Path(__file__)),not_a_deployment_candidate=True)))
        finally:sys.argv=old_argv
        path=OUT/('next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_'+tag+'_whole_hand_v1_active_ccd50/review.json')
        r=json.loads(path.read_text());summary=dict(path=path.relative_to(OUT).as_posix(),sha256=sha(path),
            gain_rad=gain,steps=r['steps'],passed=r['first_rotation_and_hold_passed'],stop=r['stop'],
            best_sampled_error_deg=min(row['error_deg'] for row in r['trace']),final=r['final'],worst=r['worst'],
            terminal_support=r['terminal_hand_support'],bundle_metadata_sha256=r['bundle_metadata_sha256'],
            actor_sha256=sha(OUT/'next_cycle/staged_shadow_bundle_v2_ccd50/actor.npz'))
        summaries.append(summary);save_json(folder/'progress.json',dict(completed=summaries,hardware_execution_allowed=False))
        print(json.dumps(dict(completed_gain=gain,steps=r['steps'],best_error_deg=summary['best_sampled_error_deg'],stop=r['stop'])),flush=True)
    original=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_whole_hand_v1_active_ccd50'
    first=(OUT/summaries[0]['path']).parent
    a=np.load(first/'research_trace.npz',allow_pickle=False)['goals'];b=np.load(original/'research_trace.npz',allow_pickle=False)['goals']
    result=dict(status='SAC_actor_gain_sensitivity_complete',rows=summaries,
        original_gain_with_projection_goals_equal_previous=np.array_equal(a,b),
        new_SAC_training_steps=0,actor_weights_unchanged=True,unmodified_native_guards_and_physical_limits=True,
        first_rotation_and_hold_passed=any(row['passed'] for row in summaries),
        complete_cycle_passed=False,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',result)
    print(json.dumps({key:value for key,value in result.items() if key!='rows'}),flush=True)


if __name__=='__main__':main()
