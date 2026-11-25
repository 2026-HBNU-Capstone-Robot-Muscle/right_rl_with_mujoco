"""Isolate a40ms health reply window; keep the legacy80ms implementation."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
saved=ROOT/'right_hand_rotation_20261028/observed90_prefix638/operator_v1_before_reply40';saved.mkdir(exist_ok=True)
for name in ('calibrated_quarter_plan_20261028.py','calibrated_quarter_gate_20261028.py','leap_calibrated_quarter_20261028.py','verify_calibrated_prefix638_20261028.py'):
    target=saved/name
    if not target.exists():target.write_bytes((ROOT/name).read_bytes())
source=(ROOT/'horizontal_sync_receive_window_20261028.py').read_text(encoding='utf-8')
source=source.replace("version='bounded_sync_receive_window_v4_single_motor_replies_20261028',packet_window_ms=80.","version='bounded_single_reply40ms_v1_20261028',packet_window_ms=40.,torque_off_packet_window_ms=80.")
source=source.replace('group=bus.sdk.GroupSyncRead(bus.port,bus.packet,address,length);window_ms=80.', 'group=bus.sdk.GroupSyncRead(bus.port,bus.packet,address,length);limit_ms=40. if length==83 else 80.;window_ms=limit_ms')
source=source.replace('window_ms=min(80.,1000*(deadline-time.monotonic()))','window_ms=min(limit_ms,1000*(deadline-time.monotonic()))')
source=source.replace('receive_window_ms=80.,health_reply_group_size=count','receive_window_ms=limit_ms,health_reply_group_size=count')
(ROOT/'horizontal_single_reply40_20261028.py').write_bytes(source.encode())
p=ROOT/'calibrated_quarter_plan_20261028.py';code=p.read_text(encoding='utf-8')
assert "operator_review'" in code
code=code.replace("operator_review'","operator_review_v2_reply40'")
code=code.replace("physical_timing_prefix638_calibration_v1_20261028","physical_timing_prefix638_calibration_v2_reply40_20261028")
code=code.replace("'calibrated_quarter_plan_20261028.py','calibrated_quarter_source_20261028.py'","'calibrated_quarter_plan_20261028.py','horizontal_single_reply40_20261028.py','single_reply40_evidence_20261028.py','single_reply_profile_20261028_054656_033637/result.json','calibrated_quarter_source_20261028.py'")
p.write_bytes(code.encode())
p=ROOT/'calibrated_quarter_gate_20261028.py';code=p.read_text(encoding='utf-8')
code=code.replace('from horizontal_sync_receive_window_20261028 import POLICY as WINDOW_POLICY','from horizontal_single_reply40_20261028 import POLICY as WINDOW_POLICY\nfrom single_reply40_evidence_20261028 import review as reply_window_review')
code=code.replace('    calibration_review()','    calibration_review();reply_window_review()')
code=code.replace("physical_timing_calibration=calibration_review(),accepted_for_operator_trial=True", "physical_timing_calibration=calibration_review(),single_reply_window_evidence=reply_window_review(),accepted_for_operator_trial=True")
p.write_bytes(code.encode())
p=ROOT/'leap_calibrated_quarter_20261028.py';code=p.read_text(encoding='utf-8')
code=code.replace('class CalibratedQuarterBus(HorizontalQuarterBus):',"from horizontal_single_reply40_20261028 import block_once as reply40_block,SCOPES as REPLY40_SCOPES\nclass CalibratedQuarterBus(HorizontalQuarterBus):\n    def _block_once(self,address,length,fields):\n        if (address,length) in REPLY40_SCOPES:return reply40_block(self,address,length,fields)\n        return super()._block_once(address,length,fields)")
code=code.replace("Same verified1Mbps,450mA/PID,2-tick steps and all health stops.","Verified1Mbps; health replies40ms each, OFF verification80ms. Same shared200ms and two clean recovery snapshots.450mA/PID/health stops unchanged.")
p.write_bytes(code.encode())
p=ROOT/'verify_calibrated_prefix638_20261028.py';code=p.read_text(encoding='utf-8').replace("('preflight','_block_once','_receive','write','checked_sample')","('preflight','_receive','write','checked_sample')")
code=code.replace("self.assertEqual(self.plan.SAMPLE_PERIOD,.075);", "from horizontal_single_reply40_20261028 import POLICY\n        self.assertEqual(POLICY['packet_window_ms'],40.);self.assertEqual(POLICY['torque_off_packet_window_ms'],80.)\n        self.assertEqual(self.plan.SAMPLE_PERIOD,.075);")
p.write_bytes(code.encode())
print('Isolated40ms health-window variant created. Original80ms bus and loaded targets preserved. No devices opened.')
