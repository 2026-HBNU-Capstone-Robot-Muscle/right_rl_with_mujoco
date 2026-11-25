"""Record reached-thumb evidence and the next screened empty preparation."""
import json
from hand_preparation_plan import PreparationPlan,FULL_TARGET
from thumb_live_start_plan import ROOT
from thumb_live_start_gate import validate_proof
from test_hand_preparation_timing import Bus,Clock,Tests

folder=ROOT/'thumb_prepare_current_empty_20261022_002034_279500'
timing=json.loads((folder/'timing_review.json').read_bytes())
assert timing['last_goal_is_final'] and timing['failed_phase']=='stage_2_fixed_hold'
assert timing['all_torque_off_verified'] and timing['settings_restored']
timing['thumb_target_reached']=True
timing['requested_hold_complete']=False
timing['visual_review']=dict(images=['camera/frame_00333.jpg','camera/frame_00615.jpg','camera/frame_00622.jpg'],
    observation='Thumb configuration changes into the prepared pose by final-goal hold. Selected frames do not certify contact-free motion throughout.',physical_clearance_verified=False)
(folder/'timing_review.json').write_text(json.dumps(timing,indent=2),encoding='utf-8')
fixture=ROOT/'right_hand_preparation_20261022/finger_fixture'
proof=validate_proof(PreparationPlan(fixture/'request.json'),fixture/'screen.json')
integration=json.loads((fixture/'windows_integration.json').read_bytes())
assert integration['actual_windows_wsl_subprocess_passed']
clock=Clock();bus=Bus(clock);replay,error=Tests().execute(bus,clock)
assert error is None and replay['status']==PreparationPlan.STATUS
ready=dict(plan_version=PreparationPlan.PLAN_VERSION,launcher='leap_hand_prepare_current_empty.cmd',
    target_ticks=FULL_TARGET.tolist(),goal_updates=704,max_step_ticks=2,
    processing_included_in_period=True,sample_period_s=.05,on_limit_s=55,
    delayed_fake_bus_replay_on_seconds=clock.t-replay['enabled_at'],
    fake_read_delay_ms=31,fake_camera_delay_ms=12,fake_sleep_jitter_ms=10,
    static_cases=len(proof['static_screens']),dynamic=proof['dynamic_sensitivity'],
    native_ranges_unchanged=True,tests_passed=45,windows_wsl_integration_passed=True,
    hardware_executed=False,physical_full_preparation_verified=False,physical_cube_rotation_verified=False,
    next_user_task='Cube OUT. Leave hand as it is, thumb/middle in camera. New launcher, terminal SPACE once; screen, countdown, roots/thumb/fingers, hold, OFF.')
(fixture.parent/'fingers_ready.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
note=f'''## CURRENT 2026-10-22 00:20 시험: 엄지 목표 도착, 유지 중 시간 제한 / 다음 손가락 준비

- `thumb_prepare_current_empty_20261022_002034_279500`는 608개 목표를 모두 보냈고 최종 목표에서 4개 fresh 읽기가 있었다. 마지막 엄지 실제 `[2593,3091,3085,2707]`, 목표 `[2601,3100,3074,2708]`, 오차 `[-8,-9,+11,-1]`틱(약 0.09~0.97°). 목표 도착은 확인했지만 요청된 유지 전체는 끝나지 않았으므로 원래 `stopped` 결과를 변경하지 않음.
- 중단은 `stage_2_fixed_hold`의 55초 제한. 마지막 읽기 t=54.984s, 유지 표본 기간 약 0.266s. 최저 4.1V, 최대 69mA, 최대 추종 0.04142rad. 16개 OFF/설정 복원 및 OFF 끝점 안정 확인. 사진에서 엄지 준비 자세 변화 확인, 전 구간 무접촉 보증은 아님.
- 실제 읽기 중앙값 31ms, 대부분 전체 주기 94ms. 50ms sleep을 읽기/화면/쓰기 뒤에 추가한 시간 계산이 제한보다 경로를 늦췄다. 새 현재각도 준비 실행은 처리 시간을 50ms 주기에 포함. 목표 생략/따라잡기 쓰기/자동 재전송 없음, 각 목표 앞 fresh health 읽기 유지. 55초 제한 확대 없음.
- 다음 사용자 작업은 `leap_hand_prepare_current_empty.cmd`로 빈손 전체 준비 시험. 현재 OFF 각도 안정 확인과 경로 검사 후 뿌리 → 엄지 소폭 조정 → 검지/중지/약지 준비 자세 → 유지 → OFF/복원. 큐브 없이 손 모양 그대로, SPACE 한 번. 엄지가 도착한 앞 동작만 별도로 반복하도록 요청하지 않음.
- 다음 704개 목표, 최대 2틱. 손가락 이동 때 엄지 목표 고정 및 실제 오차 20틱 제한. 손가락에도 20회 fresh 읽기에서 진행 없음 + 64틱 이상 오차일 때 다음 목표를 막는다. 전류/PID/전압/추종/카메라 보호 유지.
- 실측 OFF 표본의 원래 관절 범위 MuJoCo: 67개 시작 표본에서 자기 겹침/받침 겹침/관절·제어 범위 위반 0. gain 0.5/1/1.5 최대 추종 약 0.0646/0.0326/0.0219rad, 동역학 자기/받침 겹침 0. Windows→WSL→결과 해시 검증 통과. 실제 실행 때 현재 자세를 다시 검사하며 모델·모터 교정이나 실물 성공 증명은 아님.
- fake read31ms + camera12ms + sleep jitter10ms를 넣은 전체 실행/유지/OFF/복원 재생은 약 {ready['delayed_fake_bus_replay_on_seconds']:.2f}s ON으로 55초 내 통과. 회귀 및 새 손가락/시간 테스트 45개 통과. AI는 실물 모터를 구동하지 않음.
- 근거: 완료 폴더 `timing_review.json`, `right_hand_preparation_20261022/finger_fixture/windows_integration.json`, `fingers_ready.json`.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;content=path.read_text(encoding='utf-8')
    if note.splitlines()[0] not in content:path.write_text(note+content,encoding='utf-8')
path=ROOT/'right_hand_start_20261019/objective_and_schedule.json'
state=json.loads(path.read_text(encoding='utf-8'))
state['latest_user_hardware_trial']=dict(timestamp='2026-10-22T00:20:34+09:00',status=timing['status'],
    evidence='thumb_prepare_current_empty_20261022_002034_279500/timing_review.json',
    thumb_target_reached=True,requested_hold_complete=False,all_torque_off_verified=True)
state['current_user_task']=ready['next_user_task']
state['current_blocker']='Thumb preparation target reached; final hold was cut short by old timing. Processing-inclusive pacing and next EMPTY finger preparation are screened; physical full preparation/cube rotation pending.'
state['latest_offline_review']='right_hand_preparation_20261022/finger_fixture/windows_integration.json'
state['next_trial_review']='right_hand_preparation_20261022/fingers_ready.json'
path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(ready,ensure_ascii=False,indent=2))
