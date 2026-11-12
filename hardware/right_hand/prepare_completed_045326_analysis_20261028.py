"""Make independent saved-data analysis/export entries for the new full run."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
dest=ROOT/'right_hand_rotation_20261028/completed_quarter_045326';dest.mkdir(exist_ok=True)
for source,target in (
    ('inspect_completed_quarter_assets_20261028.py','inspect_completed_045326_20261028.py'),
    ('measure_same_marker_quarter_20261028.py','measure_same_marker_045326_20261028.py'),
    ('export_real_quarter_video_20261028.py','export_real_quarter_045326_20261028.py')):
    code=(ROOT/source).read_text(encoding='utf-8')
    assert 'horizontal_quarter_trial_20261028_012305_894520' in code
    code=code.replace('horizontal_quarter_trial_20261028_012305_894520','horizontal_quarter_trial_20261028_045326_636152')
    code=code.replace('completed_quarter_012305','completed_quarter_045326')
    (ROOT/target).write_bytes(code.encode('utf-8'))
print('Independent saved-frame analysis and export entries ready. No devices opened.')
