"""Close the red capture task and record the one missing physical dimension."""
import datetime as dt,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026';VIS=ROOT/'cube_red_green_20261026_215059_362527'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    protected={ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',ROOT/'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',ROOT/'loaded_repeat_20261026_170932_163649/result.json':'1722388e82f512b9f7fa24d60a75ef8320c59435c61d5f6117c6d7fec70a794d'}
    assert all(sha(p)==s for p,s in protected.items());saved=json.loads((VIS/'result.json').read_bytes());review=json.loads((VIS/'spin_invariant_pose_review.json').read_bytes());outline=json.loads((VIS/'outer_face_marker_diagnostic.json').read_bytes());seed=json.loads((OUT/'yaw_after_flip_camera_seed_v1/seed_screen.json').read_bytes());geometry=json.loads((OUT/'face_homography_geometry_checks.json').read_bytes());stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    assert saved['status']=='cube_red_green_capture_complete' and saved['saved_frames']==18 and not saved['motors_accessed'] and saved['motor_commands_sent']==0
    assert seed['model_seed_count']==0 and all(v['passed'] for v in geometry['checks'])
    action=dict(kind='measure_red_center_marker_side',motor_power='OFF',robot_hand_base_and_book='unchanged',cube='on table, outside hand',measurement='RED face centre ID2 black outer square side in mm; exclude white margin',reason='GREEN ID5 was the earlier approximately10.7mm measurement. The red marker is approximately9.8mm in an independent55mm-face homography; do not assume all faces share one marker size.',new_photo_required=False,motor_program_required=False,request_sent_asynchronously=True)
    studies=[]
    for name in ('five_seconds_per_stage','five_seconds_per_stage_forward'):
        p=OUT/'original_full_loaded_v1'/name/'review.json';r=json.loads(p.read_bytes());studies.append(dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p),cases=len(r['cases']),initial_valid_cases=sum(v['initial_valid'] for v in r['cases']),conditional_yaw_prefixes=sum('conditional_yaw_prefix' in v for v in r['cases']),native_goal_bounds=r['native_goals'],model_self_overlap_at_original_start_mm=r['static_first_rejected']['observation']['self_overlap_mm'],model_self_overlap_max_mm=r['static_max_self_overlap_mm'],model_study_is_not_proof_that_historical_physical_motion_failed=True))
    payload=dict(timestamp=stamp,status='red_green_capture_complete_red_marker_dimension_pending',physical_forward_flip_successes=2,horizontal_quarter_turn_verified=False,horizontal_motion_ready=False,source_folder=VIS.name,source_sha256=sha(VIS/'result.json'),saved_frames=18,frames_with_all_red_green_anchor=sum(p['all_three_ids'] for p in review['profiles']),
        original_equal_size_centered_marker_pose_accepted_frames=sum(p['multi_face_candidates'] for p in review['profiles']),capture_success_does_not_mean_geometry_fit_success=True,repeat_capture_requested=False,camera_settings_restored=saved['camera_restoration']['restored_original_readbacks'],
        annotation_based_marker2_size_mm=outline['faces']['2']['side_mean_mm'],annotation_based_marker2_face_offset_mm=outline['faces']['2']['face_center_offset_uv_mm'],annotation_geometry_not_physical_measurement=True,rounded_bevel_and_annotation_uncertainty_remain=True,green_marker_measurement_mm_remains_user_value=10.7,
        independent_perspective_distortion_checks=geometry['checks'],camera_derived_seed_screen='right_hand_rotation_20261026/yaw_after_flip_camera_seed_v1/seed_screen.json',camera_derived_seed_count=0,observed_model_cube_overlap_mm=[v['initial_contacts']['max_cube_overlap_mm'] for v in seed['candidates']],unverified_metric_pose_not_used_for_motion=True,
        original_full_sequence_studies=studies,original_sequence_file_unchanged=True,motor_programs_and_health_guards_unchanged=True,live_vision_configuration_unchanged=True,assistant_motor_commands_sent=0,real_devices_opened_during_review=False,
        next_user_action=action,next_user_command=None,next_user_command_ready=False,next_motion_command_ready=False,physical_measurement_not_yet_received=True,protected_hashes={p.relative_to(ROOT).as_posix():sha(p) for p in protected},hardware_execution_allowed=False)
    write(OUT/'red_green_reference_completed_review_20261026.json',payload)
    for name in ('horizontal_next_step_20261026.json','current_camera_and_yaw_review_20261026.json'):
        p=OUT/name;r=json.loads(p.read_bytes());r.update(timestamp=stamp,status=payload['status'],red_green_capture_complete=True,red_green_capture_source=VIS.name,repeat_capture_requested=False,next_user_action=action,next_user_command=None,next_user_command_ready=False,next_motion_command_ready=False,horizontal_loaded_execution_ready=False,latest_review='right_hand_rotation_20261026/red_green_reference_completed_review_20261026.json');write(p,r)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        p=ROOT/name;r=json.loads(p.read_bytes());r.update(timestamp=stamp,next_user_command=None,next_user_command_ready=False,next_user_command_photo_only=False,next_user_command_camera_only=False,next_user_command_empty_only=False,rotation_in_next_command=False,next_motion_command_ready=False,horizontal_loaded_execution_ready=False,next_user_action=action,next_action='Measure RED ID2 black outer square side once with motors OFF; camera capture already complete, no motor run requested.',latest_camera_and_yaw_review='right_hand_rotation_20261026/red_green_reference_completed_review_20261026.json');write(p,r)
    block='''## CURRENT 2026-10-26: 빨강/초록 촬영 완료, 빨강 ID2 실제 크기 확인 필요

- `cube_red_green_20261026_215059_362527`의 18장을 확인했다. 빨강 ID2, 초록 ID5, 손목 ID41은 7장에서 동시에 읽혔고 카메라 설정은 원래 읽기값으로 복원됐다. 촬영은 정상 완료다. 같은 촬영을 다시 요청하지 않으며 실제 장치를 열거나 모터 명령을 보낸 적은 없다.
- 두 마커를 모두 10.7mm이고 면 정중앙이라고 둔 기존 오프라인 모델은 이 자료의 두 면 자세를 승인하지 못했다. 두 면의 코드가 읽힌 것과 3D 모델이 맞는 것은 구분한다. 크기/각도 비교에서도 하나의 공통 마커 크기를 바로 확정할 수 없어 치수나 라이브 검출 설정을 덮어쓰지 않았다.
- 55mm 큐브의 보이는 면 외곽을 수동으로 표시하고, 렌즈 왜곡을 제거한 평면 변환으로 크기와 중심 위치를 따로 계산했다. 빨강 ID2는 약 9.80mm, 면 중심에서 약 2.58mm 벗어난 것으로 추정됐다. 주석 좌표 오차와 둥근 모서리 영향이 남아 있으므로 캘리퍼스 실측이나 자동 캘리브레이션 완료가 아니다. 초록 면 결과도 이전 사용자 실측 10.7mm와 차이가 있어 그 값을 자동 교체하지 않았다. 독립적인 3D 투영과 렌즈 왜곡을 적용한 합성 검사에서는 크기와 중심 오차가 0.001mm 미만이었으나 실제 외곽 표시의 정확도를 검증한 것은 아니다.
- 이 잠정 빨강 보정을 기존 실물 종료 사진에 적용하면 추정 높이는 손바닥 높이에 가까워졌지만, 종료 관절 모델에서는 엄지 하우징과 약 3~6mm의 겹침이 남았다. 여섯 종료 위치 후보 모두 재파지 출발 자세로 채택하지 않았다. 마커 크기, 마커의 부착 위치, 손 좌표 등록 및 충돌 형상의 오차를 아직 분리하지 못한다. 이 추정값으로 실제 회전 목표를 만들거나 충돌 기준을 넓히지 않았다.
- 예전 원본 7자세 시퀀스는 큐브를 원래 P0부터 둔 상태로도 비교했다. 손목 쪽과 손가락 쪽 가정 배치를 각각 12개 확인했지만 현재 모델의 P0에서 이미 검지/중지 하우징과 손바닥/손끝 겹침이 있었고 배치가 안정적으로 성립하지 않았다. 원본 목표는 범위 안이지만 현 모델에서 곧바로 실행 후보로 채택할 수 없다. 이전 실제 회전 경험이 틀렸다고 판정한 것이 아니며, 원본 파일이나 성공한 수직 동작을 수정하지 않았다.
- 다음 사용자 작업은 추가 촬영이나 모터 실행이 아니라 **빨강 ID2 검은 정사각형 한 변(mm)** 확인 한 가지다. 이전 10.7mm는 초록 ID5에 대한 답이었다. 모터 전원을 끄고 큐브는 책상에 둔 채 검은 테두리 바깥 기준으로 재며 흰 여백은 제외한다. 질문을 비동기로 보냈고 답은 아직 받지 않았다. 손, 본체, 책은 그대로 두면 된다. 자동 실행용 다음 명령은 비웠고 수평 회전 실행 준비는 계속 false다.
- 근거는 `red_green_reference_completed_review_20261026.json`, 촬영 폴더의 `outer_face_marker_diagnostic.json`, `yaw_after_flip_camera_seed_v1/seed_screen.json`, `original_full_loaded_v1/*/review.json`이다. 실물 수직 성공 총 2회, 수평 90도와 임의 방향 회전 미검증 상태를 유지한다. 전압/전류/추종 기준, 성공 코드, 원본 시퀀스 및 전시 문구의 해시를 보존했다.

<!-- END RED REFERENCE DIMENSION REVIEW 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8-sig');marker='<!-- END RED REFERENCE DIMENSION REVIEW 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        p.write_text(block+old.replace('## CURRENT ','## PREVIOUS ',1),encoding='utf-8')
    assert all(sha(p)==s for p,s in protected.items());print(json.dumps(dict(capture_complete=True,saved_frames=18,repeat_capture=False,next_action='measure RED ID2 black side',new_motor_command_ready=False,protected_files_unchanged=True)),flush=True)
if __name__=='__main__':main()
