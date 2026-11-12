"""Isolated EMPTY waypoint/checker; native ranges and loaded path untouched."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
saved=ROOT/'right_hand_rotation_20261028/observed90_prefix638/operator_v3_before_empty_margin';saved.mkdir(exist_ok=True)
for name in ('calibrated_quarter_plan_20261028.py','calibrated_quarter_gate_20261028.py','screen_calibrated_quarter_20261028.py'):
    if not (saved/name).exists():(saved/name).write_bytes((ROOT/name).read_bytes())
code=(ROOT/'screen_horizontal_empty_recovery_20261027.py').read_text(encoding='utf-8')
code=code.replace("ROOT=Path(__file__).resolve().parent","from calibrated_empty_ring_margin_20261028 import PHASE as RING_MARGIN,rows as margin_rows\nROOT=Path(__file__).resolve().parent")
start=code.index('    def plan(self,raw):');end=code.index('\n\n\ndef prepare()',start)
code=code[:start]+'    def plan(self,raw):\n        return margin_rows(self,raw)'+code[end:]
code=code.replace('(ENTRY,RING_CLEAR,ROOTS,OPEN,SETTLE)','(ENTRY,RING_CLEAR,RING_MARGIN,ROOTS,OPEN,SETTLE)')
(ROOT/'screen_calibrated_empty_margin_20261028.py').write_bytes(code.encode())
p=ROOT/'screen_calibrated_quarter_20261028.py';code=p.read_text(encoding='utf-8').replace('import screen_horizontal_empty_recovery_20261027 as empty','import screen_calibrated_empty_margin_20261028 as empty');p.write_bytes(code.encode())
p=ROOT/'calibrated_quarter_plan_20261028.py';code=p.read_text(encoding='utf-8')
code=code.replace('BasePlan,ROOT,PREFIX_PHASES,PHASE_MAP','BasePlan,ROOT,PREFIX_PHASES as BASE_PREFIX_PHASES,PHASE_MAP')
code=code.replace('from new_pair_grasp_plan_20261027 import NewPairGraspPlan','from new_pair_grasp_plan_20261027 import NewPairGraspPlan\nfrom calibrated_empty_ring_margin_20261028 import PHASE as RING_MARGIN,rows as margin_rows\nPREFIX_PHASES=(*BASE_PREFIX_PHASES,RING_MARGIN)')
code=code.replace('operator_review_v3_precise_two_full','operator_review_v4_empty_margin').replace('physical_timing_prefix638_calibration_v3_precise_two_full_20261028','physical_timing_prefix638_calibration_v4_empty_margin_20261028')
code=code.replace("'calibrated_quarter_plan_20261028.py','calibrated_precise_clock_20261028.py'","'calibrated_quarter_plan_20261028.py','calibrated_empty_ring_margin_20261028.py','screen_calibrated_empty_margin_20261028.py','calibrated_precise_clock_20261028.py'")
code=code.replace('prefix=serial_roots(a,[(p,q) for p,q in NewPairGraspPlan.plan(self,raw) if p in PREFIX_PHASES])','prefix=margin_rows(self,raw)')
code+='\n    def frozen_for_phase(self,phase):\n        if phase==RING_MARGIN:return np.array([i for i in range(16) if i!=14])\n        return super().frozen_for_phase(phase)\n    def progress_for_phase(self,phase):\n        if phase==RING_MARGIN:return np.array([14])\n        return super().progress_for_phase(phase)\n'
p.write_bytes(code.encode())
p=ROOT/'calibrated_quarter_gate_20261028.py';code=p.read_text(encoding='utf-8')
code=code.replace('from hand_motor_mapping import SIM_TO_MOTOR_IDS','from hand_motor_mapping import SIM_TO_MOTOR_IDS\nfrom calibrated_empty_ring_margin_20261028 import review as margin_review')
code=code.replace('single_reply_window_evidence=reply_window_review(),accepted_for_operator_trial=True','single_reply_window_evidence=reply_window_review(),empty_ring_pip_margin=margin_review(plan),accepted_for_operator_trial=True')
code=code.replace('    thumb_book_review(plan,empty)\n    return empty','    thumb_book_review(plan,empty);margin_review(plan)\n    return empty')
p.write_bytes(code.encode())
print('Native limits preserved. New EMPTY ring-PIP margin and its independent checker are ready for model review.')
