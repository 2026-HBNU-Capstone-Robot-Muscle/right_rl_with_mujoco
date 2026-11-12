"""Record the completed physical segment and next reviewed operator task."""
import json
from pathlib import Path
from thumb_live_start_plan import ROOT,REFERENCE,PLAN_VERSION,CurrentPlan
from thumb_live_start_gate import validate_proof

folder=REFERENCE.parent
r=json.loads((folder/'review.json').read_text(encoding='utf-8'))
assert r['status']=='empty_thumb_continuation_complete' and r['torque_off'] and r['restored']
r['visual_review']=dict(images=['camera/frame_00151.jpg','camera/frame_00314.jpg','camera/frame_00333.jpg'],
    observation='Thumb moves during the powered segment and changes again after OFF. No human hand is visible in these selected frames. These frames do not certify clearance throughout motion.',
    complete_motion_contact_verified=False)
(folder/'review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
fixture=ROOT/'right_hand_preparation_20261021/fresh_start_fixture'
integration=json.loads((fixture/'windows_integration.json').read_bytes())
assert integration['actual_windows_wsl_subprocess_passed'] and integration['hardware_commands_sent']==0
proof=validate_proof(CurrentPlan(fixture/'request.json'),fixture/'screen.json')
ready=dict(plan_version=PLAN_VERSION,launcher='leap_thumb_prepare_current_empty.cmd',
    tests_passed=38,windows_wsl_integration_passed=True,
    fixture_static_cases=len(proof['static_screens']),native_ranges_unchanged=True,
    current_pose_will_be_measured_and_screened_at_run=True,
    fixture_is_not_live_pose=True,hardware_executed=False,
    approved_non_thumb_family='Open-finger bending/rotation within 32 ticks of completed 19:43 trial; roots within admission bounds; every actual start still requires native screening.',
    remaining_thumb_target=[2601,3100,3074,2708],max_goal_step_ticks=2,on_limit_s=55,
    current_user_task='Cube OUT, leave fingers as they are, show thumb/middle. Run new launcher; terminal SPACE once. Wait for path check and countdown.',
    physical_clearance_verified=False,full_preparation_verified=False,physical_cube_rotation_verified=False)
(fixture.parent/'thumb_current_ready.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
note='''## CURRENT 2026-10-21: 엄지 구간 완주 / 현재 OFF 자세에서 남은 준비 구간 검사

- 사용자 실행 `thumb_continuation_empty_20261021_194300_889550` 완료. 16개 토크 OFF 및 설정 복원, 카메라 334프레임. 엄지 실제 이동 ID1 +28.477°, ID2 +30.498°, ID3 -32.520°, ID4 +24.434°. 최저 4.1V, 최대 79mA, 최대 추종 0.05216rad. 전체 준비/큐브 회전 성공으로 표시하지 않음.
- OFF 후 실제 변화 ID4 -394틱(-34.63°), ID3 -42틱, 뿌리 -68/-53/-41틱. 두 OFF 표본 사이에도 ID3 -27틱, ID4 +23틱 등이 있어 `final_off_reference_valid=False`. 사용자도 전원/토크 OFF 후 중력 처짐 가능성을 보고. 저장 OFF 표본을 안정된 현재 자세로 간주하지 않음.
- 새 `leap_thumb_prepare_current_empty.cmd`: 현재 OFF 각도 3회가 2틱 이내로 안정됨을 확인 → 해당 시작각/경로를 WSL MuJoCo로 검사 → fresh 각도 재확인 → 3초 후 뿌리 및 남은 엄지 준비 → 유지 → OFF/복원. 큐브 없이 기존 손 모양 그대로, 터미널 SPACE 한 번. 새 사진 또는 수동 각도 맞추기 요청 없음.
- 현재는 검지/중지/약지의 굽힘·회전이 기존 열린 자세 근처인 경우를 지원. 모든 임의 자세를 처리한다고 주장하지 않음. 엄지는 과거 ON 각도 일치 대신 남은 이동 상한 및 실제 경로 검사로 판단. 뿌리/엄지 외 목표 고정, 갱신 최대 2틱. 더 긴 608개 목표 경로에 한해 ON 제한 55초; 전류/PID/전압/추종/실측 진행/카메라 보호 유지.
- 저장된 OFF 표본을 사용한 Windows→WSL 호출 및 실제 결과 검증 완료. 67개 ±4틱 시작 표본: 자기 겹침/원래 관절·제어 범위 위반 0. 원래 범위를 바꾸지 않은 동역학 gain 0.5/1/1.5의 최대 추종 약 0.0655/0.0331/0.0222rad; 최대 받침 겹침 약 0.0195mm. 이 표본은 실시간 안정 시작각이나 물리적 무접촉 증명이 아니다. 실제 실행 때 현재 각도로 다시 검사해야만 쓰기가 가능.
- 경로 실패/모델 한계 위반/결과 해시 불일치/검사 중 Q·시간초과/시뮬레이션 중 자세 변화는 모터 쓰기 차단. ON 중 오류와 사용자 취소는 OFF·복원. 기존 회귀 포함 38개 테스트 통과. AI가 모터를 구동한 적 없음.
- 앞서 사용량 한도로 거절됐던 오프라인 시뮬레이션은 같은 승인 절차로 재시도해 실행 완료. 현재 해당 차단은 해소됨.
- 근거: 완료 폴더 `review.json`, `right_hand_preparation_20261021/fresh_start_fixture/windows_integration.json`, `thumb_current_ready.json`.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    file=ROOT/name;content=file.read_text(encoding='utf-8')
    if note.splitlines()[0] not in content:file.write_text(note+content,encoding='utf-8')
file=ROOT/'right_hand_start_20261019/objective_and_schedule.json'
state=json.loads(file.read_text(encoding='utf-8'))
state['latest_user_hardware_trial']=dict(timestamp=r['timestamp'],status=r['status'],
    evidence=str((folder/'review.json').relative_to(ROOT)).replace('\\','/'),cube_rotation_verified=False)
state['last_completed_user_hardware_trial']=state['latest_user_hardware_trial']
state['current_user_task']=ready['current_user_task']
state['current_blocker']='Remaining EMPTY thumb preparation now uses fresh stable OFF start and per-run native model screening. Physical completion, full finger preparation and cube reorientation remain unverified.'
state['latest_offline_review']='right_hand_preparation_20261021/fresh_start_fixture/windows_integration.json'
state['next_trial_review']='right_hand_preparation_20261021/thumb_current_ready.json'
file.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(ready,ensure_ascii=False,indent=2))
