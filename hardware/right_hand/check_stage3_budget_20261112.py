"""Calculate existing time limits from saved read-only data; no device access."""
import json
from pathlib import Path
import numpy as np
from work_grip_stage2_plan_20261112 import WorkGripStage2Plan, REQUEST, validate_candidate
from calibrated_empty_ring_margin_20261028 import rows as prefix_rows

ROOT = Path(__file__).resolve().parent
source = ROOT / 'cycle_test_readonly_20261112_050645_574736/result.json'
r = json.loads(source.read_bytes()); s = sorted(r['snapshots'][-1]['motors'], key=lambda v: v['id'])
p = WorkGripStage2Plan(REQUEST)
raw = np.asarray([v['position_ticks'] for v in s], dtype=int)
p.center = raw.copy()
p.raw_off_ticks = raw.copy()
prefix = prefix_rows(p, raw)
stage2 = len(validate_candidate()['goals'])
grip = np.load(ROOT / 'right_hand_rotation_20261112/stage3_preparation/opposed_contact_v1/axis0_middle+1_all/research_goals.npz', allow_pickle=False)
remaining_goals = int(np.floor((p.ON_TIMEOUT-23)/p.SAMPLE_PERIOD))-len(prefix)-stage2-len(grip['goals'])
report = {'status': 'read_only_start_time_budget', 'source': source.relative_to(ROOT).as_posix(),
          'all_torque_off': all(v['torque_enabled'] == 0 for v in s), 'current_ticks': raw.tolist(),
          'empty_goals': len(prefix), 'executed_stage2_loaded_goals': stage2, 'regrasp_goals': len(grip['goals']),
          'remaining_next_turn_goals_within_existing_300s': remaining_goals,
          'remaining_next_turn_seconds': remaining_goals*.075, 'original_300s_limit_changed': False,
          'position_only_budget_calculation_not_a_fresh_safety_proof': True,
          'hardware_ready': False, 'assistant_motor_writes': 0}
out = ROOT / 'right_hand_rotation_20261112/stage3_preparation/current_start_budget.json'
out.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
print(json.dumps({k: v for k, v in report.items() if k != 'current_ticks'}))
