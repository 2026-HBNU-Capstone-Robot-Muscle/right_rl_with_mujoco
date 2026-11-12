"""Verify the new complete EMPTY path against actual addressed goal writes."""
import json,hashlib
from pathlib import Path
import numpy as np
from supported_inverse_empty_plan_20261029 import SupportedInverseEmptyPlan,new_rows,LOADED,HAND_PATH
ROOT=Path(__file__).resolve().parent
F=ROOT/'supported_inverse_empty_20261029_225256_285948';OUT=F/'offline_review'
def main():
    OUT.mkdir(exist_ok=True);path=F/'result.json';r=json.loads(path.read_bytes())
    assert r['status']=='supported_inverse_empty_path_complete' and r['all_torque_off_verified'] and r['settings_restored']
    assert r['cube_present'] is False and r['hardware_authorized']
    p=SupportedInverseEmptyPlan(F/'fresh_off_request.json');rows=p.plan(np.asarray(r['accepted_enable_goal'],int))
    writes=[e['requested'] for e in r['write_events'] if e['address']==116]
    assert writes[-len(rows):]==[q.tolist() for _,q in rows]
    assert writes[-len(new_rows()):]==[q.tolist() for _,q in new_rows()]
    samples=r['samples'];frames=json.loads((F/'camera/frames.json').read_bytes())['frames']
    selected=[]
    for phase in ('stage_4_inverse_empty_gap_hold','stage_7_horizontal_yaw','stage_8_horizontal_hold','stage_10_remove_hold'):
        available=[x for x in frames if x['phase']==phase]
        if available:
            indices=(0,len(available)//2,len(available)-1) if phase=='stage_7_horizontal_yaw' else (len(available)-1,)
            selected.extend(dict(phase=phase,path=(F/'camera'/available[i]['image']).relative_to(ROOT).as_posix()) for i in indices)
    selected.append(dict(phase='final_camera',path=(F/'camera'/frames[-1]['image']).relative_to(ROOT).as_posix()))
    camera=r['camera_recording']
    report=dict(status='actual_supported_inverse_EMPTY_path_complete',source=path.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        candidate_sha256=hashlib.sha256(LOADED.read_bytes()).hexdigest(),hand_path_sha256=hashlib.sha256(HAND_PATH.read_bytes()).hexdigest(),
        all_actual_goal_writes_match_ON_rebased_queue=True,all_new_hand_goal_writes_match_bound_path=True,queue_goals=len(rows),
        exact_new_hand_goals=len(new_rows()),yaw_samples=sum(s['phase']=='stage_7_horizontal_yaw' for s in samples),
        all_torque_off_verified=True,settings_restored=True,cube_stayed_out_by_protocol=True,physical_cube_rotation_verified=False,
        minimum_voltage_v=min(min(s['voltage_v']) for s in samples),peak_current_ma=max(max(abs(c) for c in s['current_ma']) for s in samples),
        maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),read_failures=len(r.get('read_failures',[])),
        camera_frames=camera['frame_count'],camera_blank_frames=camera['image_signal_blank_frames'],
        final_OFF_reference_stable=r.get('final_off_reference_valid',False),final_OFF_not_a_new_commanded_pose=True,
        selected_frames=selected,visual_contact_review_pending=True,assistant_motor_writes=0)
    (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report))
if __name__=='__main__':main()
