"""Measured load evidence; current is not a calibrated cube force sensor."""
import hashlib,json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;F=ROOT/'four_reply_yaw_20261029_052443_385204';O=F/'offline_review';O.mkdir(exist_ok=True)
source=F/'result.json';r=json.loads(source.read_bytes());old=json.loads((ROOT/'calibrated_quarter_trial_20261028_174723_390729/result.json').read_bytes())
yaw=[s for s in r['samples'] if s['phase']=='stage_7_horizontal_yaw'];previous=[s for s in old['samples'] if s['phase']=='stage_7_horizontal_yaw'];pairs=[]
lookup={tuple(s['goal']):s for s in previous}
for s in yaw[::30]+yaw[-1:]:
    a=lookup.get(tuple(s['goal']))
    if a:pairs.append(dict(t=s['t'],goal_ID12=s['goal'][11],actual_ID12=s['actual'][11],current_ID12_ma=s['current_ma'][11],
        earlier_actual_ID12=a['actual'][11],earlier_current_ID12_ma=a['current_ma'][11],goal_vector_matches_all16=True))
last=yaw[-1];error=last['goal'][11]-last['actual'][11]
rows=[]
for i in range(16):
    rows.append(dict(motor_id=i+1,current_peak_ma=max(abs(s['current_ma'][i]) for s in r['samples']),
        yaw_current_peak_ma=max(abs(s['current_ma'][i]) for s in yaw),yaw_tracking_peak_rad=max(s['tracking_rad'][i] for s in yaw)))
summary=dict(status='measured_middle_root_load_increase',source=F.name,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    stop=r['error'],stop_phase=r['failed_phase'],last_yaw_sample=len(yaw),same_motor12_is_middle_root=True,
    stop_ID12_error_ticks=error,stop_ID12_error_degrees=error*360/4096,stop_ID12_current_ma=last['current_ma'][11],
    previous_success_ID12_peak_ma=max(abs(s['current_ma'][11]) for s in old['samples']),goal_current_cap_ma=r['enable_snapshots'][-1]['current_cap'][11],
    minimum_voltage_v=min(min(s['voltage_v']) for s in r['samples']),communication_failures=len(r.get('read_failures',[])),
    camera_blank_frames=r['camera_recording']['image_signal_blank_frames'],all_torque_OFF_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
    cube_removed_confirmed_by_user=True,physical_contact_force_calibrated=False,excessive_grip_force_proven=False,
    possibilities=['cube/contact geometry load','internal contact opposition','mechanical or cable resistance'],
    unchanged_empty_prefix_test_required_to_separate_cube_load=True,matched_target_comparisons=pairs,joint_loads=rows,
    official_goal_current_explanation='https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#goal-current102',
    current_mode='5: Goal Current is a current limit, not a measured cube force',assistant_motor_writes=0)
(O/'grip_load_review.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k not in ('matched_target_comparisons','joint_loads')},ensure_ascii=False))
