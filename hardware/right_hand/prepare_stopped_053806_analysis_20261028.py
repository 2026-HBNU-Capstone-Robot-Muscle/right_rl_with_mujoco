"""Independent saved-frame analysis of the interrupted repeat; no devices."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder='calibrated_quarter_trial_20261028_053806_737439';dest='stopped_calibrated_053806'
(ROOT/'right_hand_rotation_20261028'/dest).mkdir(exist_ok=True)
for source,target in (('measure_same_marker_045326_20261028.py','measure_stopped_calibrated_053806_20261028.py'),('trace_observed_rotation_045326_20261028.py','trace_stopped_calibrated_053806_20261028.py'),('export_real_quarter_045326_20261028.py','export_stopped_calibrated_053806_20261028.py')):
    p=ROOT/target
    if p.exists():raise RuntimeError('Do not overwrite independent analysis')
    code=(ROOT/source).read_text(encoding='utf-8').replace('horizontal_quarter_trial_20261028_045326_636152',folder).replace('completed_quarter_045326',dest)
    p.write_bytes(code.encode())
print('Stopped-run analysis entries ready. Original controller proof sources unchanged.')
