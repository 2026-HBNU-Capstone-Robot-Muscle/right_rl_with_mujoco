import json
from pathlib import Path

p=Path('right_hand_rotation_20261024/original_sequence_clearance/CAD_forward_quarter_turn_evaluation.json')
d=json.loads(p.read_bytes())
for r in d['runs']:
    c=r['matched_duration_unchanged_goal_control']
    print(r['friction'],r['mode'],r['native_gain_scale_hypothesis'],
        'tracking',r['maximum_joint_tracking_rad'],'self',r['maximum_hand_self_model_overlap_mm'],
        'support',r['maximum_hand_support_model_overlap_mm'],
        'control',c['final']['ninety_target_error_deg'],c['final']['retained'],
        'force-limited',r['native_actuator_force_limits_enabled'])
for p in [Path('right_hand_rotation_20261024/original_sequence_review/visualized_replay.json'),
    Path('pose_pair_readonly_20261024_230555_418562/result.json')]:
    d=json.loads(p.read_bytes());print(p,'keys',list(d))
    if 'placement' in d:print(d['placement'])
