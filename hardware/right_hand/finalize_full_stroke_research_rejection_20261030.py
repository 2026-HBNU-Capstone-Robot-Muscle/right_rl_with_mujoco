import json
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/full_reverse_stroke_research';r=json.loads((O/'nominal/review.json').read_bytes());c=r['cases'][0]
report=dict(status='rejected_for_inverse_rotation_goal_despite_safety_probe_pass',safety_case_passed=c['passed'],modeled_forward_deg=c['forward_from_placement_deg'],modeled_inverse_deg=c['inverse_from_forward_hold_deg'],
    target_inverse_deg=-90,rotation_goal_passed=False,hardware_execution_allowed=False,geometry_force_and_health_bounds_unchanged=True,
    not_eligible_for_hardware_due_to_composed_duration=True,one_nominal_case_only=True,source_review='nominal/review.json',devices_opened=False)
(O/'outcome.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report))
