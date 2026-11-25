"""Record verified empty preparation and the next user placement task."""
import json
import numpy as np
from prepared_placement_plan import SOURCE,PlacementPlan,ROOT
from thumb_live_start_gate import validate_proof

folder=SOURCE.parent;r=json.loads(SOURCE.read_bytes())
review=json.loads((folder/'review.json').read_text(encoding='utf-8'))
assert r['status']=='empty_hand_preparation_complete' and r['all_torque_off_verified'] and r['settings_restored']
goal=np.array(r['final_on_state']['goal']);actual=np.array(r['final_on_state']['position'])
review.update(empty_full_preparation_motor_pose_verified=True,full_thumb_preparation_verified=True,
    maximum_final_error_deg=float(max(abs(goal-actual))*360/4096),
    final_hold_samples=sum(s['phase']=='stage_2_fixed_hold' for s in r['samples']),
    last_sample_elapsed_on_s=r['samples'][-1]['t'],
    visual_review=dict(images=['camera/frame_00520.jpg','camera/frame_00531.jpg'],
        observation='Prepared empty finger configuration visible; finger posture changes after torque OFF. Selected frames do not certify contact-free motion throughout.',
        complete_motion_contact_verified=False),
    cube_placement_verified=False,physical_cube_rotation_verified=False)
(folder/'review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8')
fixture=ROOT/'right_hand_preparation_20261022/placement_fixture'
proof=validate_proof(PlacementPlan(fixture/'request.json'),fixture/'screen.json')
integration=json.loads((fixture/'windows_integration.json').read_bytes())
assert integration['actual_windows_wsl_subprocess_passed']
ready=dict(plan_version=PlacementPlan.PLAN_VERSION,launcher='leap_cube_place_capture.cmd',
    tests_passed=68,static_cases=len(proof['static_screens']),windows_wsl_integration_passed=True,
    native_ranges_unchanged=True,fixture_is_relaxing_off_sample_not_live_start=True,
    current_state_screened_again_at_run=True,
    user_sequence=['Cube OUT, FIRST terminal SPACE: empty preparation.',
                   'PLACE prompt: put cube in hand, clear hands, SECOND SPACE: save photo.',
                   'REMOVE prompt: remove cube, THIRD SPACE: OFF and restore.'],
    marker_gate=False,registration_verified=False,cube_rotation_executed=False,hardware_executed=False)
(fixture.parent/'placement_ready.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
note=f'''## CURRENT 2026-10-22 00:47 시험: 빈손 전체 준비 완료 / 큐브 배치 촬영 준비

- `hand_prepare_current_empty_20261022_004728_219622`는 704개 목표와 마지막 20개 유지 읽기까지 완료, OFF/설정 복원 확인. 마지막 목표 기준 16개 최대 오차 {review['maximum_final_error_deg']:.3f}°, 최저 4.2V, 최대 106mA, 최대 이동 오차 0.09204rad. 마지막 유지 읽기 t≈{review['last_sample_elapsed_on_s']:.2f}s. 카메라 532프레임. 이는 빈손 모터 준비 자세의 실제 도착 확인이며 큐브 회전 성공은 아님.
- OFF 직후 검지 PIP/중지 PIP/약지 회전 등이 계속 처짐: 저장 두 OFF 끝점에서 ID7 -94틱, ID11 -38틱, ID14 +58틱. `final_off_reference_valid=False` 유지. 그 표본을 현재 안정 자세로 간주하거나 손을 이전 각도로 수동 복원하도록 요청하지 않는다.
- 다음 사용자 작업 `leap_cube_place_capture.cmd`: 빈손으로 첫 SPACE → 현재 OFF 안정 읽기/경로 검사/3초 후 회복 준비 → PLACE 표시 후 큐브 올리고 손 치운 뒤 두 번째 SPACE 사진 → 큐브 제거 후 세 번째 SPACE OFF. 한 실행본에서 카메라·모터·배치 사진 저장. 준비 동안 큐브를 올리지 않으며 배치/제거 중에는 목표를 바꾸지 않는다.
- 실제로 준비된 손과 처진 OFF 관측의 범위를 새 admission envelope로 사용하지만, admission 자체를 무접촉 승인으로 취급하지 않음. 매 실행 현재 상태의 67개 ±4틱 경로 표본과 원래 범위의 동역학을 검사해야 모터 쓰기가 가능. 저장 표본에서는 gain 0.5/1/1.5 최대 오차 약 0.0675/0.0340/0.0229rad, 자기/받침 겹침 및 범위 위반 0. 실제 형상·동역학 교정은 여전히 미확정.
- 준비 후 고정 목표를 유지하며 배치/제거 각각 최대90초 fresh health 확인. SPACE를 motor read가 소비하지 않도록 버퍼링하고, 다음 확인은 키를 놓고 0.35초 지나야 받을 수 있음. 전압 복구 재읽기에서도 SPACE 보존 확인. ID40 가림은 정상 가능하며 이 촬영에 마커 게이트 없음. `registration_verified=False`; 정성 배치/부하 유지 자료로만 취급.
- Windows→WSL 실제 연동과 기존 두 경로 회귀 재검사 완료. 새 배치/키/카메라/실패 후 OFF 복원 및 기존 전압 복구 포함 68개 테스트 통과. AI가 실제 모터를 구동한 적 없음.
- 근거: 완료 폴더 `review.json`, `right_hand_preparation_20261022/placement_fixture/windows_integration.json`, `placement_ready.json`.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    file=ROOT/name;content=file.read_text(encoding='utf-8')
    if note.splitlines()[0] not in content:file.write_text(note+content,encoding='utf-8')
file=ROOT/'right_hand_start_20261019/objective_and_schedule.json'
state=json.loads(file.read_text(encoding='utf-8'))
state['latest_user_hardware_trial']=dict(timestamp=r['timestamp'],status=r['status'],
    evidence=str((folder/'review.json').relative_to(ROOT)).replace('\\','/'),
    all_torque_off_verified=True,empty_full_preparation_motor_pose_verified=True,cube_rotation_verified=False)
state['last_completed_user_hardware_trial']=state['latest_user_hardware_trial']
state['current_user_task']='Run leap_cube_place_capture.cmd. First SPACE empty preparation; PLACE cube and second SPACE photo; REMOVE cube and third SPACE OFF.'
state['current_blocker']='Empty full preparation motor pose verified. Cube fit/loaded pose capture next; physical reorientation and SAC integration still pending.'
state['latest_offline_review']='right_hand_preparation_20261022/placement_fixture/windows_integration.json'
state['next_trial_review']='right_hand_preparation_20261022/placement_ready.json'
file.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(empty_full_preparation_verified=True,max_final_error_deg=review['maximum_final_error_deg'],
    last_on_sample_s=review['last_sample_elapsed_on_s'],tests_passed=68,next_launcher=ready['launcher']),indent=2))
