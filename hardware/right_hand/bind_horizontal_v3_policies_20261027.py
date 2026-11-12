from pathlib import Path

path=Path(__file__).resolve().parent/'horizontal_quarter_gate_20261027.py'
source=path.read_text(encoding='utf-8')
old='off_preload_confirmation_policy=dict(PRELOAD_POLICY),'
new=old+'goal_readback_confirmation_policy=dict(GOAL_POLICY),empty_root_motion_policy=dict(ROOT_POLICY),'
if source.count(old)!=2 or source.count(new):raise ValueError('Unexpected policy binding source')
path.write_text(source.replace(old,new),encoding='utf-8')
print('Bound both new policies into review generation and validation; no device access.')
