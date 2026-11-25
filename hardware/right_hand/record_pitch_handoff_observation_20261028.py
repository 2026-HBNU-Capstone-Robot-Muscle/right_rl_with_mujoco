"""Record a bounded visual review of saved evidence. No hardware access."""
import datetime as dt
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
RUN=ROOT/'pitch_handoff_observe_20261028_221727_181336'
OUT=RUN/'offline_review'
PROGRESS=ROOT/'right_hand_rotation_20261028/two_axis_connection/current_progress.json'

def write(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def main():
    result=json.loads((RUN/'result.json').read_bytes())
    audit=json.loads((OUT/'command_audit.json').read_bytes())
    exports=json.loads((OUT/'video_exports.json').read_bytes())
    details=json.loads((OUT/'inspection_details.json').read_bytes())
    frames=json.loads((RUN/'camera/frames.json').read_bytes())['frames']
    assert result['status']=='pitch_handoff_observation_complete'
    assert audit['exact_ordered_target_match'] and result['camera_worker_stopped']
    assert result['all_torque_off_verified'] and result['settings_restored']
    assert details['unique_frame_hashes']==1997
    for item in exports['exports']:
        assert (ROOT/item['video']).is_file()
    source_sha=hashlib.sha256((RUN/'result.json').read_bytes()).hexdigest()
    assert source_sha=='782e672b732ecf4712e1fd66d0c44ffce2652799c9ddc3913c25566ae255731d'
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    observations=[
        dict(image='cube_before_pitch.jpg',observation='No cube visible in the designated placement photo; not accepted as a loaded baseline.'),
        dict(image='camera/frame_00812.jpg',observation='First recorded pitch frame is still empty.'),
        dict(image='camera/frame_00850.jpg',observation='A person is placing the cube during the early recorded pitch phase.'),
        dict(image='camera/frame_00860.jpg',observation='Cube visibly resting GREEN up / ORANGE toward wrist; placing hand is clear in this frame.'),
        dict(image='camera/frame_01450.jpg',observation='GREEN face has turned toward the wrist/camera; cube is still tipping.'),
        dict(image='camera/frame_01500.jpg',observation='RED upper face is visible as the pitch completes.'),
        dict(image='cube_pitch_hold.jpg',observation='RED up / GREEN wristward at the pitch hold.'),
        dict(image='cube_after_transfer.jpg',observation='Cube remains RED up / GREEN wristward after hand transfer, without visible fall.'),
        dict(image='camera/frame_01990.jpg',observation='Cube removed before the final recorded OFF stage.')]
    for row in observations:
        path=RUN/row['image'];assert path.is_file()
        row['sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    review=dict(status='qualitative_pitch_and_retained_transfer_observed_with_late_placement',
        timestamp=stamp,source_run=RUN.name,source_result_sha256=source_sha,
        observed_green_up_to_red_up=True,observed_green_face_toward_wrist_after_pitch=True,
        cube_retained_during_sampled_transfer=True,corrected_pitch_observation_count=1,
        transfer_observation_count=1,exact90_degrees_verified=False,
        loaded_from_first_pitch_goal_verified=False,
        placement_photo_contains_cube=False,late_placement_visible_during_early_pitch_phase=True,
        first_clear_placed_cube_image='camera/frame_00860.jpg',
        first_clear_cube_image_after_pitch_phase_first_image_s=frames[860]['camera_read_monotonic']-frames[812]['camera_read_monotonic'],
        person_stabilizes_base_visible=True,
        continuous_no_person_cube_contact_certified=False,
        sampled_motion_images_show_base_hand_clear_of_cube=True,
        continuous_cube_retention_certified=False,registered_transfer_cube_pose_verified=False,
        subsequent_yaw_executed=False,combined_pitch_plus_yaw_verified=False,
        next_yaw_allowed=False,same_trial_repeat_requested=False,
        camera_worker_stopped=True,webcam_can_be_disconnected=True,additional_camera_capture_needed_now=False,
        all_torque_off_verified=True,settings_restored=True,cube_removal_confirmed=True,
        planned_targets=audit['planned_targets'],exact_target_sequence_verified=True,
        read_failures=1,recovery_clean_reads=2,recovery_elapsed_ms=116.76440003793687,
        goals_resent_during_recovery=0,min_voltage_v=4.8,peak_current_ma=134,
        max_tracking_rad=0.11351457830353745,recorded_camera_frames=1997,blank_frames=0,
        unique_frame_hashes=1997,observations=observations,videos=exports['exports'],
        raw_physical_certification_fields_not_modified=True,raw_recordings_not_modified=True,
        robot_commands_performed_by_this_review=0,
        next_offline_work='Reconcile actual retained cube seating before planning a connected yaw; improve placement checkpoint clarity in the next distinct operator.')
    write(OUT/'physical_review.json',review)
    state=json.loads(PROGRESS.read_bytes())
    previous=OUT/'progress_before_saved_run_review.json'
    if not previous.exists():write(previous,state)
    state.update(status=review['status'],timestamp=stamp,latest_user_execution=RUN.name,
        next_user_command=None,next_user_command_ready=False,next_user_action_required=False,
        next_loaded_action='No next action queued. Review actual retained seating before any connected yaw.',
        latest_physical_review=(OUT/'physical_review.json').relative_to(ROOT).as_posix(),
        latest_actual_min_voltage_v=4.8,latest_actual_peak_current_ma=134,
        physical_corrected_pitch_unverified=False,physical_transfer_unverified=False,
        verification_scope='Qualitative sampled face transition and retained transfer; late initial placement; no exact-angle or full loaded-reference certification.',
        exact90_degrees_verified=False,loaded_from_first_pitch_goal_verified=False,
        registered_transfer_cube_pose_verified=False,next_yaw_forbidden=True,
        webcam_can_be_disconnected=True,additional_capture_needed_now=False,
        camera_worker_stopped=True,motors_off_verified=True,settings_restored=True,
        same_trial_repeat_requested=False,hardware_execution_performed_by_assistant=False,
        next_offline_work=review['next_offline_work'])
    write(PROGRESS,state)
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
                 'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        write(ROOT/name,state)
    note='''2026-10-28 22:17 실행 저장 자료 검토 완료. `pitch_handoff_observe_20261028_221727_181336`는 보완 뒤집기와600개 다음 자세 이동 목표, 큐브 제거, 토크OFF/설정복구까지 완주했다. 실제3120개 경로 목표가 계획과 순서대로 정확히 일치하고, 별도 사전 목표2개 외 목표 재전송은 없었다. 저장1997장은 모두 유효하며 서로 다른 파일이고 검은 화면0이다. 통신 오류1건은116.8ms 안에 완전한 정상 읽기 두 번을 확인해 복구했다. 최저4.8V, 최대134mA, 최대추종오차0.114rad다.

저장 중간 장면에서 초록 윗면/주황 손목 쪽이 빨강 윗면/초록 손목 쪽으로 바뀌었고, 손 자세를 옮긴 뒤에도 그 방향으로 큐브가 남아 있음을 관찰했다. 다만 지정 시작 사진과 첫 뒤집기 프레임에는 큐브가 없고 초반 뒤집기 구간 도중 사람이 큐브를 놓는 장면이 있다. 따라서 보완 뒤집기와 전달 유지의 정성 관찰1회로 기록하며, 첫 목표부터 큐브를 올린 동일 조건 재현이나 정확90도 및 전 구간 무보조 인증으로 세지 않는다. 원본 인증 필드와 영상은 고치지 않았다. 다음 수평 회전은 보내지 않았다.

카메라 프로세스 종료를 확인했으므로 사용자는 웹캠을 빼도 된다. 지금 추가 촬영이나 같은 실행의 반복은 필요 없다. 일반 속도 전체 영상과4배속 검토 영상, 관찰 장면, 명령 검증은 이 실행 폴더의 `offline_review`에 있다. 다음은 저장된 전달 종료 위치와 다음 파지 모델을 비교하는 오프라인 작업이다. 수직/수평 연결 및 전체 풀이 완료로 표시하지 않는다.

<!-- END PITCH HANDOFF SAVED OBSERVATION 20261028 -->

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig')
        if '<!-- END PITCH HANDOFF SAVED OBSERVATION 20261028 -->' not in old:
            path.write_text(note+old,encoding='utf-8')
    text='''웹캠은 빼도 됩니다. 새 촬영은 지금 필요 없습니다.

이번 저장 영상에서 초록 윗면이 빨강 윗면으로 바뀌고, 다음 손 자세로 이동한 뒤에도 큐브가 유지된 것을 확인했습니다. 수평 회전은 이 실행에 포함하지 않았습니다.

다만 시작 사진에는 큐브가 없고 초반 동작이 시작된 뒤 큐브가 놓였습니다. 따라서 실제 면 전환과 전달 유지 관찰로 기록하며 처음부터 큐브를 올린 동일 조건 재현이나 정확90도 인증으로 표시하지 않습니다. 재촬영을 요청하지 않고 기존 자료로 다음 위치를 분석합니다.

- [4배속 검토 영상](pitch_and_transfer_preview_4x.mp4)
- [전체 일반 속도 영상](full_run_realtime.mp4)
- [초기 배치와 뒤집기 장면](early_placement_and_pitch.jpg)
- [뒤집기 후 자세 이동 장면](pitch_transfer_and_removal.jpg)
- [명령과 종료 검증](command_audit.json)
- [정성 관찰 및 제한](physical_review.json)

촬영1997장, 검은 화면0. 토크OFF와 설정복구를 확인했습니다. 통신 오류1건은 목표 재전송 없이 정상 읽기 두 번으로 복구했고 전체 동작을 마쳤습니다.
'''
    (OUT/'검토결과.md').write_text(text,encoding='utf-8')
    assert hashlib.sha256((RUN/'result.json').read_bytes()).hexdigest()==source_sha
    print(json.dumps(dict(status=review['status'],webcam_can_be_disconnected=True,additional_capture_needed_now=False,
        exact_ordered_targets=audit['planned_targets'],read_recovery_ms=review['recovery_elapsed_ms'],
        physical_review=(OUT/'physical_review.json').relative_to(ROOT).as_posix(),next_user_action_required=False),ensure_ascii=False))

if __name__=='__main__':main()
