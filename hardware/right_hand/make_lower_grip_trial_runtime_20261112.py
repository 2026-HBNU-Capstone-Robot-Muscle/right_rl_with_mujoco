"""Build a separate user-started lower-grip observation runtime from reviewed transport.

This creates code only. A source-bound proof and model results are still required
before the entry point can open a device. Existing execution files are preserved.
"""
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    pairs={
        'support_first_trial_plan_20261112.py':'lower_grip_trial_plan_20261112.py',
        'support_first_trial_gate_20261112.py':'lower_grip_trial_gate_20261112.py',
        'support_first_trial_capture_20261112.py':'lower_grip_trial_capture_20261112.py',
        'screen_support_first_current_20261112.py':'screen_lower_grip_current_20261112.py',
        'leap_cycle_support_first_transfer_20261112.py':'leap_cycle_lower_grip_20261112.py',
        'leap_cycle_support_first_transfer.cmd':'leap_cycle_lower_grip.cmd',
        'screen_support_first_models_20261112.py':'screen_lower_grip_models_20261112.py',
        'finalize_support_first_trial_20261112.py':'finalize_lower_grip_trial_20261112.py',
        'screen_support_first_runtime_guards_20261112.py':'screen_lower_grip_runtime_guards_20261112.py',
        'test_support_first_trial_20261112.py':'test_lower_grip_trial_20261112.py'}
    replacements={
        'SupportFirstTrial':'LowerGripTrial','support_first_trial':'lower_grip_trial',
        'screen_support_first_current':'screen_lower_grip_current','screen_support_first_models':'screen_lower_grip_models',
        'screen_support_first_runtime_guards':'screen_lower_grip_runtime_guards',
        'leap_cycle_support_first_transfer':'leap_cycle_lower_grip'}
    for old,new in pairs.items():
        path=ROOT/new
        if path.exists():raise RuntimeError('Preserve existing runtime: '+new)
        text=(ROOT/old).read_text(encoding='utf-8-sig')
        for a,b in replacements.items():text=text.replace(a,b)
        path.write_text(text,encoding='utf-8')
    print('Separate runtime source created; proof required, no devices opened.')

if __name__=='__main__':main()
