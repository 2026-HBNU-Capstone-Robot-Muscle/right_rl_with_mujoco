"""Refresh only the three explicitly approved team source/doc paths."""
import shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'team_integration_20261112/right_rl_with_mujoco/hardware/right_hand'
for name in ('tip_first_transfer_20261112.py','test_tip_first_transfer_20261112.py'):shutil.copyfile(ROOT/name,folder/name)
shutil.copyfile(ROOT/'team_integration_20261112/README.md',folder/'README.md')
print('Three team review files synchronized.')
