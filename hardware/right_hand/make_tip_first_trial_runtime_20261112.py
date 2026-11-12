"""Create a separate runtime for the user-requested tip-first transfer probe."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
pairs={
 'lower_grip_trial_plan_20261112.py':'tip_first_trial_plan_20261112.py',
 'lower_grip_trial_gate_20261112.py':'tip_first_trial_gate_20261112.py',
 'lower_grip_trial_capture_20261112.py':'tip_first_trial_capture_20261112.py',
 'screen_lower_grip_current_20261112.py':'screen_tip_first_current_20261112.py',
 'screen_lower_grip_models_20261112.py':'screen_tip_first_models_20261112.py',
 'screen_lower_grip_runtime_guards_20261112.py':'screen_tip_first_runtime_guards_20261112.py',
 'finalize_lower_grip_trial_20261112.py':'finalize_tip_first_trial_20261112.py',
 'leap_cycle_lower_grip_20261112.py':'leap_cycle_tip_first_transfer_20261112.py',
 'leap_cycle_lower_grip.cmd':'leap_cycle_tip_first_transfer.cmd',
 'test_lower_grip_trial_20261112.py':'test_tip_first_trial_20261112.py'}
for old,new in pairs.items():
    target=ROOT/new
    if target.exists():raise RuntimeError('Preserve existing '+new)
    text=(ROOT/old).read_text(encoding='utf-8-sig')
    for a,b in (('LowerGripTrial','TipFirstTrial'),('lower_grip_trial','tip_first_trial'),('screen_lower_grip','screen_tip_first'),('leap_cycle_lower_grip','leap_cycle_tip_first_transfer')):
        text=text.replace(a,b)
    target.write_text(text,encoding='utf-8')
print('Separate tip-first runtime sources prepared; no devices opened.')
