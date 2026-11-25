import json,hashlib
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;r=json.loads((R/'leap_placement_alignment_capture_result.json').read_bytes());folder=Path(r['camera_recording']['folder']).parent
assert folder.resolve().is_relative_to(R);O=folder/'offline_review';O.mkdir(exist_ok=True)
ref=json.loads((R/'single_forward_trial_20261030_052636_068402/result.json').read_bytes());q=np.asarray(r['placement_goal_ticks']);old=np.asarray(ref['placement_goal_ticks']);samples=r.get('samples',[])
events=r.get('write_events',[]);placed=[s for s in samples if s.get('phase') in ('place_cube','remove_cube')];frames=json.loads((folder/'camera/frames.json').read_bytes())['frames']
def extrema(key):
 values=[np.asarray(s[key]) for s in samples if key in s]
 return [float(np.min(values)),float(np.max(values))] if values else None
summary=dict(source=folder.relative_to(R).as_posix(),status=r['status'],error=r.get('error'),all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],cube_placement_confirmed=r.get('cube_placement_confirmed'),cube_removal_confirmed=r.get('cube_removal_confirmed'),post_placement_goal_writes=r.get('post_placement_goal_writes'),same_commanded_open_pose_as_reference=bool(np.array_equal(q,old)),placed_joint_difference_from_reference=(np.asarray(r['placement_position_ticks'])-np.asarray(ref['placement_position_ticks'])).tolist(),placement_photo=r['placement_photo'],prepared_empty_photo=r.get('prepared_empty_photo'),camera=r['camera_recording'],last_frame=frames[-1],health_extrema={k:extrema(k) for k in ('voltage_v','current_ma','temperature_c','tracking_rad')},read_recoveries=len(r.get('read_failures',[])),samples=len(samples),position_writes=sum(e.get('address')==116 for e in events),position_sample_schema=list(samples[0]) if samples else [],source_result_sha256=hashlib.sha256((folder/'result.json').read_bytes()).hexdigest(),motor_writes_by_review=0)
(O/'capture_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps(summary,ensure_ascii=False,indent=2))
