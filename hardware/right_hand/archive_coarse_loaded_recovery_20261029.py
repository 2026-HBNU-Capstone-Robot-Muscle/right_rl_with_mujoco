"""Preserve the first EMPTY screen before binding the refined model sources."""
from adaptive_yaw_loaded_plan_20261029 import ROOT,OUT
source=OUT/'empty_recovery_screen';target=OUT/'empty_recovery_screen_before_refinement'
assert source.resolve().is_relative_to(ROOT.resolve()) and target.resolve().is_relative_to(ROOT.resolve())
if target.exists():raise RuntimeError('Archive already exists')
source.rename(target)
print('Preserved initial recovery review; no device access.')
