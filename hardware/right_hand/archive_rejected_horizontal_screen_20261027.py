from pathlib import Path
root=Path(__file__).resolve().parent
source=(root/'right_hand_rotation_20261027/horizontal_quarter_trial_v3/empty_recovery_screen').resolve()
target=source.with_name('empty_recovery_screen_clearance64_rejected').resolve()
source.relative_to(root);target.relative_to(root)
if not source.is_dir() or target.exists():raise ValueError('Unexpected archive paths')
source.rename(target)
print('Retained rejected serial-only screen; fresh screen will include short support clearance.')
