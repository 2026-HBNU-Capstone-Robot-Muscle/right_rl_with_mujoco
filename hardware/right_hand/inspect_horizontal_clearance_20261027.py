import json
from pathlib import Path
root=Path(__file__).resolve().parent
report=json.loads((root/'right_hand_rotation_20261027/horizontal_quarter_trial_v3/empty_recovery_screen/review.json').read_bytes())
for front in (-15,-10,-5):
    rows=[v for v in report['static_cases'] if v['book_front_mm']==front]
    print(json.dumps(dict(front=front,first=rows[0],clear_bounds=[min(v['root_clear_sample'] for v in rows),max(v['root_clear_sample'] for v in rows)],largest_clear_goal=[max(v['root_clear_goal_ticks'][i] for v in rows) for i in range(3)])))
