"""Record a qualitative physical forward-flip milestone; preserve raw evidence."""
import datetime as dt,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'loaded_thumb_rotation_20261026_135418_442474'
OUT=SOURCE/'visual_review'
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    raw=read(SOURCE/'result.json');commands=read(SOURCE/'command_completion_review.json')
    index=read(OUT/'video_source_index.json');video=read(OUT/'video_export_review.json')
    assert commands['status']=='exact_loaded_command_sequence_complete' and commands['all_goals_exactly_once']
    assert commands['source_sha256']==sha(SOURCE/'result.json')==index['result_sha256']
    assert video['source_index_sha256']==sha(OUT/'video_source_index.json')
    assert video['encoded_sha256']==sha(OUT/video['encoded_file'])
    assert raw['cube_removal_confirmed'] and raw['all_torque_off_verified'] and raw['settings_restored']
    assert raw['camera_recording']['image_signal_blank_frames']==0
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    review=dict(timestamp=stamp,status='physical_forward_quarter_turn_demonstrated_once',
        success_scope='Qualitative target face orientation in one supervised real trial; not calibrated angle accuracy or arbitrary-direction control.',
        source=SOURCE.name,result_sha256=sha(SOURCE/'result.json'),command_completion_review_sha256=sha(SOURCE/'command_completion_review.json'),
        before_image='cube_before_turn.jpg',before_sha256=sha(SOURCE/'cube_before_turn.jpg'),
        after_image='cube_after_turn.jpg',after_sha256=sha(SOURCE/'cube_after_turn.jpg'),
        observed_start='GREEN top / ORANGE toward wrist ID41',observed_end='RED top / GREEN toward wrist ID41',
        physical_quarter_turn_demonstrated=True,whole_cube_orientation_changed=True,
        visual_target_orientation_reached=True,precise_3D_angle_measured=False,
        exact_90_degree_accuracy_verified=False,repeatability_verified=False,successful_trials=1,
        arbitrary_direction_rotation_verified=False,horizontal_quarter_turn_verified=False,cube_solving_verified=False,
        reviewed_selected_frames=len(index['selected_frames']),selected_frames=index['selected_frames'],
        human_cube_contact_observed_in_reviewed_frames=False,person_finger_supporting_base_visible=True,
        continuous_human_contact_exclusion_certified=False,
        final_target_pose_visible_during_hold=True,latest_on_final_pose_not_safe_OFF_reference=not raw['final_off_reference_valid'],
        video=video['encoded_file'],video_sha256=video['encoded_sha256'],recorded_span_seconds=index['recorded_span_seconds'],
        stored_rotation_frames=index['frame_count'],maximum_recorded_frame_gap_seconds=index['maximum_recorded_frame_gap_seconds'],
        playback='original recorded timestamp pace; reconstructed from saved still frames, no sound',
        minimum_voltage_v=commands['minimum_voltage_v'],maximum_current_ma=commands['maximum_current_ma'],maximum_tracking_rad=commands['maximum_tracking_rad'],
        all_torque_off_verified=True,settings_restored=True,raw_result_physical_flip_flag_preserved=raw['physical_flip_verified'],
        raw_recording_not_modified=True,hardware_commands_sent=0,
        next_action='Repeat the same leap_loaded_thumb_rotation.cmd once to check reproduction. First SPACE EMPTY; place GREEN up/ORANGE toward wrist only at PLACE; second SPACE starts rotation; remove cube before third SPACE. No motor-gain/current/voltage or path change is requested.')
    write(OUT/'physical_flip_review.json',review)
    ready_path=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/loaded_thumb_root_fixture_v4_portable_proof/readiness_review.json'
    ready=read(ready_path)
    assert all(sha(Path(path))==expected for path,expected in ready['protected_files_verified'].items())
    ready.update(latest_physical_forward_flip_review=(OUT/'physical_flip_review.json').relative_to(ROOT).as_posix(),
        latest_physical_forward_flip_review_sha256=sha(OUT/'physical_flip_review.json'),
        physical_quarter_turn_demonstrated=True,physical_ninety_degree_rotation_verified=True,
        physical_rotation_verification_basis='Qualitative RED-up/GREEN-wrist target; exact angle not measured.',
        exact_ninety_degree_accuracy_measured=False,successful_physical_trials=1,
        repeatability_verified=False,next_action=review['next_action'])
    write(ready_path,ready)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json',
                 'right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json',
                 'right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        path=ROOT/name;r=read(path)
        r.update(timestamp=stamp,loaded_thumb_rotation_readiness=ready,physical_forward_flip_review=review,
            physical_quarter_turn_demonstrated=True,physical_ninety_degree_rotation_verified=True,
            physical_rotation_verification_basis='Qualitative face orientation in one trial; not exact calibrated angle.',
            precise_ninety_degree_angle_measured=False,physical_forward_flip_successful_trials=1,
            repeatability_verified=False,arbitrary_direction_rotation_verified=False,
            next_user_command='leap_loaded_thumb_rotation.cmd',next_user_command_ready=True,
            next_user_command_empty_only=False,next_user_command_has_loaded_motion=True,rotation_in_next_command=True,
            next_action=review['next_action'])
        write(path,r)
    block='''## CURRENT 2026-10-26: 실물 수직 뒤집기 목표 면 도달 1회 확인

- `loaded_thumb_rotation_20261026_135418_442474`가 큐브를 올린 전체 경로를 완주했다. 목표 2,193개가 순서대로 한 번씩 전송됐고, 저장된 목표 읽기값은 모두 명령과 일치했다. 큐브 배치와 제거가 확인됐으며 토크 OFF와 설정 복원도 완료됐다. 최저 전압 3.6V, 최대 전류 152mA, 최대 추종 오차 약 0.133rad다. 3.6V 관측 두 번은 각각 63ms와 125ms 안에 두 번의 정상 전압 읽기로 회복했으며 그 확인 중 새 목표는 전송하지 않았다. 기준을 낮추지 않고 완주한 실행이다.
- 실물 전후 사진과 회전 중 추출 프레임에서 초록 위/주황 손목 방향의 시작 자세가 빨강 위/초록 손목 방향으로 바뀌고 마지막 고정 구간에 유지되는 것을 확인했다. 이번 실물 90도급 수직 뒤집기 한 번은 목표 면 전환의 시각적 기준으로 성공이다. 정밀한 3D 각도 오차를 교정 측정한 것은 아니다. 검토한 장면에서는 사람 손가락이 본체를 지지하고 큐브를 직접 조작하는 모습은 보이지 않는다. 모든 프레임의 사람 접촉 부재를 인증한 것은 아니다.
- 실제 촬영 프레임 777장을 촬영 시각에 맞춰 H.264 영상 `loaded_thumb_rotation_20261026_135418_442474/visual_review/actual_forward_flip_realtime.mp4`로 묶었다. 촬영 구간 100.297초, 저장 프레임 사이 최대 간격 약 0.172초이며 검정 프레임이 없다. 출력은 약 100.4초, 1280×720, 30fps이고 시간 정합을 위해 같은 원본 프레임을 반복 표시했다. 속도 변경과 프레임 보간 및 원본 이미지 변형은 하지 않았다. 소리는 없다. 원본 전체 카메라 기록은 2,959장이다.
- 이는 수직 방향 한 번의 실물 성공이며, 반복성이나 모든 방향 회전, 수평 90도 회전 및 큐브 풀이 전체 성공은 아직 확인하지 않았다. 다음 사용자 작업은 같은 `leap_loaded_thumb_rotation.cmd` 한 번으로 재현 여부를 확인하는 것이다. 경로와 힘 및 중단 기준을 바꾸지 않는다. 첫 SPACE는 빈손, PLACE 후 초록 위/주황 손목 방향으로 큐브를 놓고 두 번째 SPACE, REMOVE 후 큐브를 뺀 다음 세 번째 SPACE다. 모터 OFF 뒤 손가락이 내려온 끝값은 안정된 출발 기준으로 재사용하지 않으며 다음 실행은 새 OFF 값을 읽어 준비 경로를 검사한다.
- 근거: 실행의 `command_completion_review.json`, `visual_review/physical_flip_review.json`, `video_source_index.json`, `video_export_review.json`, `actual_rotation_contact_sheet.jpg`와 실제 영상. 원래 실행 결과의 `physical_flip_verified=false`는 영상 자동 판정을 하지 않는 기록기의 원래 값으로 보존했고, 별도 시각 검토 결과에 이번 성공을 기록했다. 전시 문구 및 원본 시퀀스와 관절 범위 파일은 그대로다. 이번 분석 중 실제 모터 쓰기와 토크 및 설정 변경은 없다.

<!-- END PHYSICAL FLIP SUCCESS CURRENT 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END PHYSICAL FLIP SUCCESS CURRENT 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT ','## PREVIOUS ',1);path.write_text(block+old,encoding='utf-8')
    assert sha(SOURCE/'result.json')==review['result_sha256']
    assert all(sha(Path(path))==expected for path,expected in ready['protected_files_verified'].items())
    print(json.dumps(dict(status=review['status'],video=str(OUT/video['encoded_file']),next_command='leap_loaded_thumb_rotation.cmd',physical_trials=1,exact_angle_measured=False)),flush=True)
if __name__=='__main__':main()
