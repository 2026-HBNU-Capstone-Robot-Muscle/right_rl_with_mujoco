"""Source-bound inspection of the user's completed loaded inverse run."""
import hashlib,json
from pathlib import Path
import numpy as np
from gapped_inverse_plan_20261029 import GappedInversePlan,loaded_rows
ROOT=Path(__file__).resolve().parent;F=ROOT/'gapped_inverse_trial_20261029_184252_272166';O=F/'offline_review';O.mkdir(exist_ok=True)

def main():
    path=F/'result.json';r=json.loads(path.read_bytes());p=GappedInversePlan(F/'fresh_off_request.json')
    rows=p.plan(np.asarray(r['accepted_enable_goal'],int))
    written=[e['requested'] for e in r['write_events'] if e['address']==116]
    assert written[-len(rows):]==[q.tolist() for _,q in rows]
    assert written[-len(loaded_rows()):]==[q.tolist() for _,q in loaded_rows()]
    assert r['status']=='gapped_inverse_observation_complete' and r['all_torque_off_verified'] and r['settings_restored']
    assert r['cube_removal_confirmed'] and not r['cube_present']
    samples=r['samples'];camera=r['camera_recording']
    report=dict(status='completed_loaded_inverse_angle_review_pending',source=path.relative_to(ROOT).as_posix(),
        source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        all_actual_goal_writes_match_ON_rebased_queue=True,queue_goals=len(rows),
        all_loaded_goals_match_reviewed_tape=True,loaded_goals=len(loaded_rows()),
        inverse_yaw_samples=sum(s['phase']=='stage_7_horizontal_yaw' for s in samples),
        all_torque_off_verified=True,settings_restored=True,cube_removal_confirmed=True,
        last_saved_frame='camera/frame_01897.jpg',
        last_frame_review='Robot hand empty in inspected saved frame; operator fingers remain on base',
        endpoint_OFF_reference_stable=r.get('final_off_reference_valid',False),
        minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
        peak_current_ma=max(max(abs(c) for c in s['current_ma']) for s in samples),
        maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),
        read_failures=len(r.get('read_failures',[])),
        all_read_failures_recorded_recovered=all(e.get('recovered',False) for e in r.get('read_failures',[])),
        ON_fresh_read_recovery_confirmations=len(r.get('read_recovery_confirmation_events',[])),
        recovery_durations_ms=[e['elapsed_ms'] for e in r.get('read_recovery_confirmation_events',[])],
        camera_frames=camera['frame_count'],camera_blank_frames=camera['image_signal_blank_frames'],
        qualitative_cube_reverse_motion_seen=True,
        visual_observation='GREEN remains up; WHITE front moves to the side and ORANGE comes forward',
        exact_robot_axis_90_verified=False,physical_angle_quantification_pending=True,
        direct_cube_hand_assistance_not_seen_in_selected_photos=True,
        continuous_no_assistance_certified=False,base_was_steadied_by_operator=True,
        assistant_motor_writes=0)
    (O/'physical_execution_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report))

if __name__=='__main__':main()
