from pathlib import Path
import json
ROOT=Path(__file__).resolve().parent
review=json.loads((ROOT/'right_hand_rotation_20261109/middle_probe_reconstruction/review.json').read_bytes())
for sample,row in review['checkpoints'].items():
    print(json.dumps({'sample':sample,'qpos_diff':row['maximum_qpos_difference_from_original'],'worst_gaps':row['upper_gaps_mm'][:8],'contacts':row['contacts']},ensure_ascii=False))
actual=json.loads((ROOT/'right_hand_rotation_20261109/middle_probe_actual_review/review.json').read_bytes())
for phase,row in actual['phases'].items():
    if phase in ('stage_6_grasp_hold','stage_7_middle_fold','stage_8_middle_hold','stage_8b_middle_return','stage_8c_restored_hold'):
        print(json.dumps({'phase':phase,'median_middle_actual':row['median_actual_last10'][8:12],'goal_middle':row['last_goal'][8:12],'peak_current':row['max_abs_current_ma']},ensure_ascii=False))
