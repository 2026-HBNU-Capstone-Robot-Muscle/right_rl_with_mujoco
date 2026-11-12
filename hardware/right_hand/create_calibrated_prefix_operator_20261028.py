"""Create an isolated physical-timing calibration variant; baseline untouched."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
gate=(ROOT/'horizontal_quarter_gate_20261027.py').read_text(encoding='utf-8')
gate=gate.replace('from horizontal_quarter_plan_20261027 import (','from calibrated_quarter_plan_20261028 import (')
gate=gate.replace('checker=ROOT/\'screen_horizontal_quarter_20261027.py\'',"checker=ROOT/'screen_calibrated_quarter_20261028.py'")
old="if not isinstance(nominal.get('yaw_before_release_deg'),(int,float)) or not math.isfinite(nominal['yaw_before_release_deg']) or not 85<=nominal['yaw_before_release_deg']<=95 or not bounded(nominal,{'yaw_tilt_change_deg':15}):raise RuntimeError('Nominal model does not approach the quarter-turn target')"
new="if not isinstance(nominal.get('yaw_before_release_deg'),(int,float)) or not math.isfinite(nominal['yaw_before_release_deg']) or not 60<=nominal['yaw_before_release_deg']<=65 or not bounded(nominal,{'yaw_tilt_change_deg':15}):raise RuntimeError('Calibration nominal dynamics differ from reviewed response')\n    calibration_review()"
assert old in gate;gate=gate.replace(old,new)
gate=gate.replace("status='horizontal_quarter_operator_review',accepted_for_operator_trial=True", "status='physical_timing_calibration_operator_review',physical_timing_calibration=calibration_review(),accepted_for_operator_trial=True")
gate=gate.replace('expected=dict(accepted_for_operator_trial=True,','expected=dict(physical_timing_calibration=calibration_review(),accepted_for_operator_trial=True,')
gate=gate.replace('from horizontal_serial_link_20261028 import metadata as link_review','from horizontal_serial_link_20261028 import metadata as link_review\nfrom calibrated_quarter_source_20261028 import review as calibration_review')
gate=gate.replace('"""Check immutable model evidence before operator-triggered setting writes."""','"""Separate calibration gate: unchanged safety criteria, measured prefix timing.\n\nThis experimental route has a62-degree nominal model response. Its physical\n90-degree aim comes from the preceding saved real run; neither model accuracy\nnor physical90-degree completion of this shorter route is certified.\n"""')
(ROOT/'calibrated_quarter_gate_20261028.py').write_bytes(gate.encode())
screen=(ROOT/'screen_horizontal_quarter_20261027.py').read_text(encoding='utf-8')
screen=screen.replace('from horizontal_quarter_plan_20261027 import','from calibrated_quarter_plan_20261028 import').replace('from horizontal_quarter_gate_20261027 import','from calibrated_quarter_gate_20261028 import')
screen=screen.replace('LARGE TURN REVIEW PASSED','CALIBRATION ROUTE REVIEW PASSED')
(ROOT/'screen_calibrated_quarter_20261028.py').write_bytes(screen.encode())
command=(ROOT/'leap_horizontal_quarter_trial.cmd').read_text(encoding='utf-8-sig')
assert 'leap_horizontal_quarter_20261027.py' in command
(ROOT/'leap_calibrated_quarter_trial.cmd').write_bytes(command.replace('leap_horizontal_quarter_20261027.py','leap_calibrated_quarter_20261028.py').encode())
print('Independent calibration gate, checker and user-started command created. Baseline unchanged; no devices opened.')
