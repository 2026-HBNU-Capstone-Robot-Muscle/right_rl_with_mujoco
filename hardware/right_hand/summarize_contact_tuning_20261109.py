from pathlib import Path
import json
ROOT=Path(__file__).resolve().parent
r=json.loads((ROOT/'right_hand_rotation_20261109/lower_grip_contact_tuning/review.json').read_bytes())
for row in r['cases']:
    if row.get('native_goal_rejected'):continue
    d=row['dynamic'];t=d['settled_tail']
    if abs(row['cumulative_yaw_from_probe_deg'])<5 and t['minimum_upper_gap_sampled_75ms_mm']>.3:
        print(json.dumps({'name':row['name'],'cumulative_yaw':row['cumulative_yaw_from_probe_deg'],'local_yaw':d['regrasp_yaw_change_deg'],'tilt':d['final_up_tilt_deg'],'tail':t,'task':d['whole_grip_task_passed']}))
