"""Verify the executed Cube route, preserve failures and measured limits."""
import json,hashlib,datetime as dt
from pathlib import Path
import numpy as np
from supported_inverse_trial_plan_20261029 import SupportedInverseTrialPlan,LOADED
ROOT=Path(__file__).resolve().parent;F=ROOT/'supported_inverse_trial_20261029_231345_341471';O=F/'offline_review'
def main():
    r=json.loads((F/'result.json').read_bytes());p=SupportedInverseTrialPlan(F/'fresh_off_request.json')
    rows=p.plan(np.asarray(r['accepted_enable_goal'],int));written=[e['requested'] for e in r['write_events'] if e['address']==116]
    assert written[-len(rows):]==[q.tolist() for _,q in rows]
    assert r['status']=='supported_inverse_observation_complete' and r['all_torque_off_verified'] and r['settings_restored']
    assert r['cube_removal_confirmed'] and not r['cube_present']
    angle=json.loads((O/'side_face_rotation_review.json').read_bytes());samples=r['samples'];camera=r['camera_recording']
    assert angle['after_axis_consistent'] and angle['before_axis_vote_purity']>=.9
    comparisons=angle['comparisons'];hold=comparisons['placement_to_turn']['palm_relative_twist_p05_median_p95_deg'][1];opened=comparisons['placement_to_open']['palm_relative_twist_p05_median_p95_deg'][1]
    report=dict(status='completed_supported_cube_observed32_not90',source=(F/'result.json').relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256((F/'result.json').read_bytes()).hexdigest(),
        candidate_sha256=hashlib.sha256(LOADED.read_bytes()).hexdigest(),all_actual_goal_writes_match_ON_rebased_queue=True,queue_goals=len(rows),yaw_goals=328,
        all_torque_off_verified=True,settings_restored=True,cube_removal_confirmed=True,cube_removal_seen_in_last_saved_frame=True,
        min_voltage_v=min(min(s['voltage_v']) for s in samples),peak_current_ma=max(max(abs(c) for c in s['current_ma']) for s in samples),
        max_tracking_rad=max(max(s['tracking_rad']) for s in samples),read_failures=len(r.get('read_failures',[])),camera_frames=camera['frame_count'],camera_blank_frames=camera['image_signal_blank_frames'],
        estimated_placed_to_hold_deg=hold,estimated_placed_to_open_deg=opened,physical_inverse_90_verified=False,
        angle_review='side_face_rotation_review.json',green_axis_in_side_marker=angle['green_axis_in_ID0'],up_axis_relation_image_checked=True,
        failed_top_grid_fit_preserved=True,frame_spread_not_absolute_accuracy=True,certified_robot_registration=False,
        improvement_over_previous54_not_observed=True,assistant_motor_writes=0)
    (O/'physical_execution_and_angle_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    (O/'top_grid_analysis_rejection.json').write_text(json.dumps(dict(status='rejected_insufficient_readable_top_marker_and_grid',
        fitted_grid_frames=0,reason='No accepted ID5-labelled grid fit. The rejected method did not supply any angle or face-up calibration.',
        replacement='side_face_rotation_review.json',source_images_unchanged=True),indent=2)+'\n',encoding='utf-8')
    for rel in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        path=ROOT/rel;x=json.loads(path.read_bytes());x.update(status='supported_inverse32_observed_previous54_reference_under_revision',current_status='supported_inverse32_observed_previous54_reference_under_revision',
            timestamp=dt.datetime.now().astimezone().isoformat(),next_command=None,next_user_command=None,next_user_command_ready=False,next_user_action_required=False,
            next_user_action='Keep cube outside. Do not repeat the32deg trial; further offline work is in progress.',ready_for_user_run=False,ready_for_supervised_operator_run=False,
            latest_actual_run=F.name,latest_actual_cube_run=F.name,latest_actual_review=(O/'physical_execution_and_angle_review.json').relative_to(ROOT).as_posix(),
            last_actual_cube_rotation_review=(O/'physical_execution_and_angle_review.json').relative_to(ROOT).as_posix(),
            estimated_inverse_hold_deg=hold,estimated_inverse_after_open_deg=opened,
            inverse_cube_trial_ready=False,inverse_90_ready=False,new_supported_inverse_cube_trial_not_yet_run=False,
            best_observed_inverse_reference='gapped_inverse_trial_20261029_184252_272166/offline_review/physical_inverse_review.json',
            cube_removal_seen_in_final_saved_frame=True,assistant_motor_or_setting_writes=0)
        path.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    marker='<!-- actual_supported_inverse32_20261029_231345 -->\n'
    paragraph=(f'23:13 새 큐브 경로 전체완료/OFF/복구/제거확인,지령{len(rows)}개와회전328개가실제쓰기와일치했다. 최저{report["min_voltage_v"]:.1f}V/최대{report["peak_current_ma"]}mA/최대{report["max_tracking_rad"]:.5f}rad,읽기0/검정0. '
        f'회전은배치→고정{hold:.2f}도,배치→OPEN{opened:.2f}도로추정돼90도목표에못미쳤으며이전약54도보다작다. '
        '이번초록ID5격자법은유효0프레임으로기각보존했다. 같은흰ID0와해독된손바닥ID40을사용하고,그린스티커위치를기준으로흰마커좌표의위축(-X)을영상에서검증했다. '
        '현재재실행명령은철회했으며기존54도경로의접촉과확장을검토중이다. 마지막저장프레임에서빈손이보여제거질문은필요없다. '
        '근거: supported_inverse_trial_20261029_231345_341471/offline_review/physical_execution_and_angle_review.json\n\n')
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8-sig')
        if not text.startswith(marker):path.write_text(marker+paragraph+text,encoding='utf-8')
    print(json.dumps(report))
if __name__=='__main__':main()
