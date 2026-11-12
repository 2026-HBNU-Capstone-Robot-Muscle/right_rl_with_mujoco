"""Record saved user trial and offline findings; no device access or motion."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'prepared_cube_review_20261022'
photo=json.loads((OUT/'placement_review.json').read_text(encoding='utf-8'))
native=json.loads((OUT/'native_replay.json').read_bytes())
projected=json.loads((OUT/'projected_path_OFFLINE_ONLY.json').read_bytes())
searchpath=OUT/'clearance_search.json'
search=json.loads(searchpath.read_bytes())
# Empty-cube pose is deliberately collision-disabled and falls; it is not an
# orientation evaluation. Suppress that irrelevant free-fall value in review.
for variant in search['variants']:
    for row in variant.get('dynamic',[]):
        if not row['loaded']:
            for k in ('tip_deg','off_axis_deg','target_error_deg','shift_mm'):row[k]=None
searchpath.write_text(json.dumps(search,indent=2),encoding='utf-8')
run=ROOT/photo['source'];r=json.loads((run/'result.json').read_bytes())
review=json.loads((run/'review.json').read_text(encoding='utf-8'))
review.update(empty_full_preparation_motor_pose_verified=True,full_thumb_preparation_verified=True,
    cube_placement_photo_saved=True,cube_orientation_photo='Green up, orange toward camera.',
    actual_cube_support_surface_confirmed=False,registration_verified=False,
    physical_cube_rotation_verified=False)
(run/'review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8')
pending=dict(status='placement_complete_offline_motion_not_ready',source=photo['source'],
    source_sha256=photo['source_sha256'],prepared_empty_and_loaded_photos_reviewed=True,
    cube_ids_detected=[3,5],two_marker_fit_mean_reprojection_px=photo['cube_marker_pose_candidates'][0]['mean_reprojection_px'],
    empty_to_loaded_maximum_change_ticks=photo['max_empty_to_loaded_tick_delta'],
    cube_support_surface='Not visible beneath cube in this new prepared pose; user clarification pending.',
    historical_bridge_rejected_for_deployment=True,
    original_candidate_static_self_overlap_mm=native['static_max_self_overlap_mm'],
    revised_projection_rotated_in_synthetic_trials=False,
    physical_cube_rotation_verified=False,live_sac=False,hardware_commands_sent_by_assistant=0,
    next_user_task='State whether cube bottom rested on black palm (ID40) or root motor/link surfaces. No repeat motion/capture required.',
    next_motion_launcher=None)
(OUT/'current_status.json').write_text(json.dumps(pending,ensure_ascii=False,indent=2),encoding='utf-8')
note='''## CURRENT 2026-10-22 01:04 배치 완료 / 새 회전 경로는 오프라인 수정 중

- `prepared_placement_capture_20261022_010440_125130`: 준비 274개 목표, 배치 사진, 큐브 제거 확인, 토크 OFF/설정 복원 완료. 최저 4.4V, 최대 57mA, 최대 추종 0.04909rad. 빈손/큐브 배치 시 관절 변화 최대 5틱. `prepared_empty.jpg`, `cube_placed.jpg` 직접 검토: 초록 위/주황 카메라 쪽, 두 큐브 마커 ID3/ID5 검출. 두 면 마커 스핀 16개 조합을 비교한 최저 평균 잔차 1.106px. 사진과 마커 검출 성공이며 실물 회전 성공/3D 좌표 교정은 아님.
- 현재 empty에는 ID41만 검출, placed에는 ID3/ID5만 검출. 과거 10/13 ID40/41 관계를 옮긴 모든 평면해는 관측된 초록 윗면과 불일치(최소 기울기 약18.9°), 무검증 과거 변환은 사용하지 않음. 현재 새 자세의 큐브 바닥/실제 받침은 사진에서 가려져 있어 사용자에게 손바닥인지 뿌리 모터/마디인지 간단히 질문. 재구동이나 재촬영 요청 없음.
- 실제 배치 관절값을 native MuJoCo 시작값으로 사용하고, -64틱 엄지 준비 변화에서 기존 회전 의도로 연결하는 새 오프라인 후보 검사. 원래 관절/제어 범위 변경 없음. **큐브 위치는 저장된 합성 reset 5개이며 사진 좌표로 교정된 것이 아님.** 5개에서 약75.8~89.9° 회전이 나왔지만 하나의 추종 오차0.3509rad, 명령 자세의 손바닥/엄지 자기 겹침 최대16.10mm가 있어 실물용 경로로 채택하지 않음. 실제 동역학 겹침 약0.8~1.5mm와 정적 목표 겹침은 서로 다른 값이다.
- 엄지 끝 각도 후보17개를 오프라인 비교했으나 작은 각도 조정만으로 겹침 해결 못 함. 동역학의 실제 도달 각도에 자기 접촉 회피 최적화를 적용한 후보는 정적 겹침0이지만 일부 최적화 미수렴, 동역학 자기 겹침0.55~0.66mm 및 큐브 회전 약-1.4°로 목적 실패. 이를 SAC 성공이나 실물 실행 가능으로 보고하지 않음. 다음 경로를 기존 임계값 완화로 통과시키지 않는다.
- 런타임 모터 코드/전류/PID/보호 조건은 변경하지 않음. 조수는 카메라/모터를 연결하지 않고 저장 자료로만 분석. 새 실행 명령은 아직 없으며 사용자에게 같은 배치 촬영을 반복시키지 않음.
- 근거: `prepared_cube_review_20261022/placement_review.json`, `native_replay.json`, `clearance_search.json`, `projected_path_OFFLINE_ONLY.json`, `current_status.json`. `candidate_simulation.gif`는 고정 경로/합성 큐브 reset 시뮬레이션일 뿐 live SAC나 실물 성공 영상이 아님.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;content=p.read_text(encoding='utf-8')
    if note.splitlines()[0] not in content:p.write_text(note+content,encoding='utf-8')
p=ROOT/'right_hand_start_20261019/objective_and_schedule.json'
state=json.loads(p.read_text(encoding='utf-8'))
trial=dict(timestamp=r['timestamp'],status=r['status'],evidence=photo['source']+'/review.json',
    all_torque_off_verified=True,empty_full_preparation_motor_pose_verified=True,
    cube_placement_photo_saved=True,cube_rotation_verified=False)
state.update(latest_user_hardware_trial=trial,last_completed_user_hardware_trial=trial,
    current_user_task=pending['next_user_task'],
    current_blocker='New prepared cube support/registration uncertain; old flip intent has static self collision. Offline repair remains unsuccessful, not ready for hardware.',
    latest_offline_review='prepared_cube_review_20261022/current_status.json',next_trial_review=None)
p.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(pending,ensure_ascii=False,indent=2))
