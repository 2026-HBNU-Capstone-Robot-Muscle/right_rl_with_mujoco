"""Prepare the one-joint correction from its completed native replay."""
import hashlib
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261112/nominal_cycle_trial_preparation'
FIX=OUT/'rf16_2177_v2'; BACKUP=OUT/'before_rf16_fix_20261112'
OLD=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/initial_second_turn_hold_replay/research_goals.npz'
MODEL=ROOT/'right_hand_rotation_20261112/nominal_cycle_actual_review/rf16_cap2177_model'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
if FIX.exists() or BACKUP.exists():raise RuntimeError('Preserve correction preparation')
raw=json.loads((MODEL/'review.json').read_bytes()); old=dict(np.load(OLD,allow_pickle=False)); new=dict(np.load(MODEL/'candidate.npz',allow_pickle=False))
assert raw['passed'] and not raw['stop'] and raw['goals_done']==raw['total_goals']==5089
assert raw['source_sha256']==sha(OLD) and raw['candidate_sha256']==sha(MODEL/'candidate.npz')
assert np.array_equal(new['start'],old['start']) and np.array_equal(new['goals'][:,:15],old['goals'][:,:15])
assert np.array_equal(new['goals'][:,15],np.minimum(old['goals'][:,15],2177))
BACKUP.mkdir()
for p in list(OUT.glob('*.json'))+[ROOT/n for n in ('nominal_cycle_plan_20261112.py','nominal_cycle_gate_20261112.py','leap_cycle_nominal_two_turns_20261112.py')]:
    (BACKUP/p.name).write_bytes(p.read_bytes())
FIX.mkdir();(FIX/'candidate.npz').write_bytes((MODEL/'candidate.npz').read_bytes())
holds=[{'samples':h['samples'],'supported_samples':h['supported'],'upper_certified_samples':h['clear'],
        'minimum_upper_lower_mm':h['minimum_upper_mm'],'hold_shift_mm':h['shift_mm'],'hold_drift_deg':h['drift_deg'],
        'target_error_deg':h['target_error_deg'],'tilt_deg':h['tilt_deg']} for h in raw['holds']]
r={'status':'RF16_reduced_goal_complete_cycle_native_replay','steps':5089,'all1ms_samples':5089*75,
   'candidate_sha256':sha(FIX/'candidate.npz'),'source_files_sha256':{OLD.relative_to(ROOT).as_posix():sha(OLD),
       (MODEL/'review.json').relative_to(ROOT).as_posix():sha(MODEL/'review.json'),
       'screen_rf16_cap_20261112.py':sha(ROOT/'screen_rf16_cap_20261112.py')},
   'two_rotations_and_clear_holds_at_reached_locations_passed':raw['passed'],'actual_state_resets_after_initial_placement':0,
   'holds':holds,'seams':[],'seams_not_applicable_to_single_continuous_replay':True,
   'final180_error_deg':holds[-1]['target_error_deg'],'final_tilt_deg':holds[-1]['tilt_deg'],'worst':raw['worst'],
   'model_provenance':raw['model_provenance'],'native_guards_and_joint_bounds_unchanged':True,
   'hardware_execution_allowed':False,'physical_success':False,'RF16_goal_cap_ticks':2177,
   'all_other_goal_columns_bitwise_unchanged':True,'changed_commands':raw['changed_commands']}
(FIX/'model_review.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
manifest=json.loads((OUT/'manifest.json').read_bytes());manifest.update(source_goals_sha256=sha(FIX/'candidate.npz'),
    source_review_sha256=sha(FIX/'model_review.json'),correction='Only ID16 goal upper cap2177; all safety thresholds unchanged')
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
a=json.loads((OUT/'operator_scope_approval.json').read_bytes()); assert a['maximum_on_seconds']==500 and a['user_explicitly_approved']
a.update(previous_source_goals_sha256=a['source_goals_sha256'],source_goals_sha256=sha(FIX/'candidate.npz'),
         source_rebinding_reason='Authorized routine fix: lower only ID16 goals; prior500s approval and diagnostic scope unchanged')
(OUT/'operator_scope_approval.json').write_text(json.dumps(a,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
last=json.loads((ROOT/'nominal_two_turn_trial_20261112_055048_322766/result.json').read_bytes())['samples'][-1]
print(json.dumps({'changed_ID16_commands':raw['changed_commands'],'other_motors_unchanged':True,
                  'latest_ID16_actual':last['actual'][15],'latest_ID16_goal':last['goal'][15],
                  'corrected_goal_cap':2177,'new_candidate_sha256':sha(FIX/'candidate.npz')}))
