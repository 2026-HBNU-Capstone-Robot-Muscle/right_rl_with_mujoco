"""Package the independently checked small correction, preserving all failures.

This only prepares local files. It neither opens devices nor starts motion.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261112/nominal_cycle_trial_preparation'
RESEARCH=ROOT/'right_hand_rotation_20261112/nominal_cycle_actual_review/support_transfer_fix_v3'
FIX=OUT/'index8_support_v3';BACKUP=OUT/'before_index_support_fix_v3_20261112'
OLD=OUT/'rf16_2177_v2/candidate.npz'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def put(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def main():
    proof_path=RESEARCH/'independent_index8/model_review.json'
    proof=json.loads(proof_path.read_bytes()); source=proof_path.parent/'candidate.npz'
    assert proof['passed'] and not proof['stop'] and proof['steps']==5089 and proof['all1ms_samples']==381675
    assert proof['candidate_sha256']==sha(source) and proof['actual_state_resets_after_initial_placement']==0
    old=dict(np.load(OLD,allow_pickle=False));new=dict(np.load(source,allow_pickle=False))
    expected=old['goals'].copy();expected[:,7]+=np.rint(8*np.clip((np.arange(5089)-1799)/200,0,1)).astype(int)
    assert np.array_equal(expected,new['goals']) and np.array_equal(old['start'],new['start'])
    assert np.max(abs(np.diff(np.vstack([new['start'],new['goals']]),axis=0)))<=2
    bias_path=RESEARCH/'index_comp8_bias33/review.json';bias=json.loads(bias_path.read_bytes())
    bias_candidate=bias_path.parent/'candidate.npz';bias_arrays=np.load(bias_candidate,allow_pickle=False)
    assert np.array_equal(bias_arrays['goals'],new['goals']) and not bias['passed']
    if FIX.exists() or BACKUP.exists():raise RuntimeError('Preserve already prepared correction')
    BACKUP.mkdir()
    names=('nominal_cycle_plan_20261112.py','nominal_cycle_gate_20261112.py','leap_cycle_nominal_two_turns_20261112.py')
    for p in list(OUT.glob('*.json'))+[OUT/'operator_guide.md']+[ROOT/n for n in names]:
        (BACKUP/p.name).write_bytes(p.read_bytes())
    FIX.mkdir();(FIX/'candidate.npz').write_bytes(source.read_bytes())
    proof['source_files_sha256'].update({proof_path.relative_to(ROOT).as_posix():sha(proof_path),
        bias_path.relative_to(ROOT).as_posix():sha(bias_path),bias_candidate.relative_to(ROOT).as_posix():sha(bias_candidate)})
    proof.update(status='nominal_index8_support_correction_v3_not_physical_fix_certificate',
        tracking_bias_sensitivity_passed=False,tracking_bias33_first_hold_error_deg=bias['holds'][0]['target_error_deg'],
        tracking_bias33_upper_certified_samples=bias['holds'][0]['clear'],
        correction_is_unverified_physical_hypothesis=True)
    put(FIX/'model_review.json',proof)
    # These failures occur before the first changed command. Exact arrays, model,
    # gain and control timing are identical through each failing prefix. Retain
    # the failure by this deterministic prefix proof, not a claimed new replay.
    for gain in (.5,1.,1.5):
        parent=OLD.parent/f'hand_gain{gain:g}.json';h=json.loads(parent.read_bytes());n=h['goals_done']
        assert not h['passed'] and h['stop'] and h['source_sha256']==sha(OLD)
        assert np.array_equal(old['goals'][:n],new['goals'][:n])
        h.update(status='unchanged_failed_hand_prefix_proof_not_new_dynamic_replay',source_sha256=sha(FIX/'candidate.npz'),
            inherited_failure_source=parent.relative_to(ROOT).as_posix(),inherited_failure_sha256=sha(parent),
            exact_command_prefix_equal_through_failure=True,new_full_hand_replay_performed=False)
        put(FIX/f'hand_gain{gain:g}.json',h)
    manifest=json.loads((OUT/'manifest.json').read_bytes());manifest.update(source_goals_sha256=sha(FIX/'candidate.npz'),
        source_review_sha256=sha(FIX/'model_review.json'),correction='Only ID8: smooth0-to8tick support correction after1800; priorID16cap2177 retained',
        physical_fall_fix_confirmed=False,tracking_uncertainty_passed=False)
    put(OUT/'manifest.json',manifest)
    approval=json.loads((OUT/'operator_scope_approval.json').read_bytes())
    assert approval['user_explicitly_approved'] and approval['maximum_on_seconds']==500
    approval.update(previous_source_goals_sha256=approval['source_goals_sha256'],source_goals_sha256=sha(FIX/'candidate.npz'),
        source_rebinding_reason='User requested repair; boundedID8goal correction within native range;500s and all existing physical limits retained')
    put(OUT/'operator_scope_approval.json',approval)
    put(FIX/'change_summary.json',dict(changed_motor_IDs=[8],maximum_added_ticks=8,maximum_added_degrees=8*360/4096,
        prior_RF16_cap_retained=True,other15_goal_columns_unchanged=True,changed_commands=int(np.count_nonzero(new['goals'][:,7]!=old['goals'][:,7])),
        prefix1800_commands_unchanged=True,same5089_goals_and75ms=True,all_stop_thresholds_unchanged=True,
        nominal_model_passed=True,tracking_bias_sensitivity_passed=False,physical_fall_fix_confirmed=False,devices_opened=0))
    print('Independent nominal candidate packaged; tracking sensitivity failure retained; no device access.')

if __name__=='__main__':main()
