"""Publish the completed EMPTY comparison and the next operator-only test."""
import datetime,hashlib,json
import numpy as np
from adaptive_yaw_empty_plan_20261029 import ROOT,OUT,REQUEST,PROOF,AdaptiveYawEmptyPlan
from adaptive_yaw_empty_gate_20261029 import validate_review
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
p=AdaptiveYawEmptyPlan(REQUEST);proof=validate_review(p,PROOF)
comparison_path=ROOT/'yaw_load_empty_20261029_055917_615720/offline_review/loaded_comparison.json';comparison=read(comparison_path)
completed_path=ROOT/'yaw_load_empty_20261029_055917_615720/result.json';completed=read(completed_path)
assert completed['status']=='empty_yaw_load_comparison_complete' and completed['all_torque_off_verified'] and completed['settings_restored']
assert sha(completed_path)==comparison['empty_sha256']
fresh_path=ROOT/'fast_off_readonly_20261029_061601_083408/result.json';fresh=read(fresh_path)
assert fresh['status']=='off_snapshots_saved' and fresh['maximum_drift_ticks']<=2 and not fresh['servo_writes'] and not fresh['torque_changes']
delta=int(np.max(abs(np.asarray([s['position'] for s in fresh['snapshots']])-p.source_position())))
assert delta<=4,'Current posture requires a new model review'
protected={
    'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
    'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
    'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
    'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d',
    'horizontal_quarter_motion_runner_20261027.py':'5c8487f134264f5ef4eb9f6bdf6a465e5fb6c69945c4a91407e1b6d75cc760cb',
    'C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py':'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'}
for filename,digest in protected.items():assert sha(ROOT/filename)==digest,filename
now=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat()
ready=dict(status='EMPTY_bounded_catch_up_ready',timestamp=now,proof=PROOF.relative_to(ROOT).as_posix(),proof_sha256=sha(PROOF),
    latest_readonly=fresh_path.relative_to(ROOT).as_posix(),current_pose_difference_ticks=delta,current_recovery_static_cases=201,
    current_recovery_dynamic_cases=9,fixed_goal_hold_stress_dynamic_cases=9,software_checks_passed=11,
    software_check_sources={f:sha(ROOT/f) for f in ('test_root_lag_pause_20261029.py','test_adaptive_yaw_empty_20261029.py')},
    latest_completed_empty_result=completed_path.relative_to(ROOT).as_posix(),latest_completed_empty_sha256=sha(completed_path),
    measured_cube_comparison= comparison_path.relative_to(ROOT).as_posix(),measured_same_command_ID12=comparison['at_loaded_stop_same_command'],
    measured_empty_fixed_hold=comparison['fixed_end_hold'],steady_state_response_cause_not_proven=True,
    same_angular_targets_and_current_PID=True,yaw_targets=320,root_lag_pause_policy=proof['root_lag_pause_policy'],
    nominal_motion_seconds=proof['nominal_motion_seconds'],maximum_added_wait_seconds=20.,active_motion_limit_s=300,
    cube_stays_OUT=True,one_terminal_SPACE_only=True,no_PLACE_or_REMOVE=True,
    physical_wait_controller_success_not_yet_verified=True,loaded_controller_trial_ready=False,assistant_motor_writes=0,
    protected_hashes_unchanged=True)
(OUT/'readiness.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
guide='''中지 지연을 기다리는 제어의 빈손 시험

큐브는 처음부터 끝까지 손과 카메라 시야 밖에 둡니다. 손가락과 받침은 현재 그대로 두세요.
leap_adaptive_yaw_empty.cmd를 실행하고 터미널에서 SPACE 한 번만 누릅니다.
준비 자세, 같은 잡기 동작, 같은 회전 관절 구간320개, 최종1.5초 고정 후 자동 OFF합니다.
약2~3분입니다. PLACE나 REMOVE, 추가 SPACE는 없습니다. Q/Esc/Ctrl+C는 중단합니다.
끝나거나 멈추면 했어라고 알려주세요. 전체 로그를 붙일 필요는 없습니다.

중지 뿌리ID12의 지연이0.12rad 이상이면 마지막 목표를 유지하며 새 목표 쓰기를 보류합니다.
정상 상태에서0.10rad 이내를2회 연속 확인하면 다음 목표로 진행합니다.
대기당 최대1.2초, 합계20초이며 기존300초 동작 제한에 포함됩니다.
시간 내에 따라오지 못하면 중단하고 OFF합니다. 목표 재전송이나 자동 동작 재시도는 없습니다.
전류/PID/전압/온도/0.18rad 추종 중단/관절 진행/고정 관절 검사는 기존과 같습니다.
목표 각도와 파지 간격을 바꾸지 않았습니다. 이번 시험에는 큐브를 올리지 않습니다.

직전 빈손 비교는320개 회전 목표와 최종 고정까지 완료했고 OFF와 원래 설정 복구를 확인했습니다.
동일한16개 관절 목표에서 중지 지연은 빈손93틱(약8.2도)/162mA, 큐브가 있을 때119틱(약10.5도)/209mA였습니다.
빈손 회전 중 최대 지연은0.1795rad로 기존0.18rad 중단선에 가까웠습니다.
최종 고정 목표2421에서 실제 위치2371→2378, 오차50→43틱(약4.4→3.8도)이었습니다.
회전 중 최대 지연과 이 최종 고정값은 서로 다른 목표입니다. 고정만으로 모든 지연이 해소된다는 증거는 아닙니다.
따라서 큐브를 세게 잡는 문제만으로 설명할 수 없지만, 기구 저항이나 구동 응답 중 원인은 아직 확정하지 않았습니다.

현재 복귀201개 정적 조건과9개 동적 조건, 추가 고정 시간을 넣은 빈손9개 동적 조건, 소프트웨어11개 검사를 통과했습니다.
모든 가능한 대기 시점을 완전히 검증한 것은 아니며 실제 대기 제어의 효과는 이번 빈손 시험에서 확인합니다.
'''.replace('中지','중지')
(OUT/'실행안내.md').write_text(guide,encoding='utf-8')
command='& "'+str(ROOT/'leap_adaptive_yaw_empty.cmd')+'"'
for filename in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/filename;r=read(path)
    r.update(status=ready['status'],current_status=ready['status'],timestamp=now,
        next_command=command,next_user_command=command,next_user_command_ready=True,next_physical_command_ready=True,
        next_user_action_required=True,next_user_action='EMPTY throughout: one terminal SPACE; unchanged grasp/yaw320 with bounded ID12 waits; automatic OFF',
        next_physical_command_kind='EMPTY_BOUNDED_CATCH_UP_TIMING',next_step='Verify bounded catch-up timing in EMPTY hand before a loaded trial',
        trajectory='EMPTY preparation -> same grasp -> yaw320 with bounded waits -> hold1.5s -> OFF',cube_orientation_at_place=None,next_user_orientation=None,
        nominal_motion_seconds=ready['nominal_motion_seconds'],active_motion_limit_s=300,
        source_off_readonly=ready['latest_readonly'],latest_readonly=ready['latest_readonly'],latest_pose_difference_from_review_ticks=delta,
        proof=ready['proof'],proof_sha256=ready['proof_sha256'],software_checks_passed=11,
        latest_completed_empty_result=ready['latest_completed_empty_result'],latest_empty_load_comparison=comparison,
        next_yaw_readiness=ready,current_cube_removed_confirmed_by_user=True,
        yaw_motion_ready=False,empty_load_comparison_ready=False,adaptive_empty_yaw_ready=True,
        motors_off_verified=True,original_settings_restored_now=True,
        physical_combined_rotation_verified=False,assistant_motor_writes=0,assistant_motor_or_setting_writes=0)
    path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
notice=('2026-10-29 현재 단계: 동일 경로 빈손 비교를 완료했고, 중지 지연을 기다리는 제어의 빈손 시험을 준비했다. '
    'yaw_load_empty_20261029_055917_615720은 회전320개+최종1.5초 고정/OFF/원래 설정 복구까지 완료했다. '
    '동일한16관절 목표에서ID12는 빈손93틱(8.17도)/162mA, 큐브가 있을 때119틱(10.46도)/209mA였다. '
    '빈손에서도 회전 중 최대0.17948rad 지연이 생겨 파지력만의 문제로 볼 수 없다. '
    '최종 고정 목표2421에서 오차50→43틱(0.07670→0.06596rad)이었으며, 회전 최대 지연과 서로 다른 목표이므로 정지로 모든 지연이 해소된다고 단정하지 않는다. '
    '다음leap_adaptive_yaw_empty.cmd는 같은 각도/힘/PID와 건강/진행 기준을 유지하며, yaw중ID12 지연≥0.12rad일 때 목표 쓰기를 보류하고0.10rad이내2회 정상 읽기 후 진행한다. '
    '대기당1.2초/합계20초, 기존300초 제한에 포함, 새 목표 쓰기/재전송 없이 읽기만 하며, 따라오지 못하면OFF한다. '
    '현재 복귀201정적+9동적과 고정 대기 스트레스9동적, 소프트웨어11검사를 통과했다. '
    '실물 대기 제어는 아직 미검증이다. 큐브는 끝까지 밖에 두고 터미널SPACE1회,약2~3분,자동OFF한다. '
    '현재 관절 읽기는 모터 쓰기0건이고 모델출발점 차이최대'+str(delta)+'틱이다.\n\n')
for filename in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/filename;previous=path.read_text(encoding='utf-8')
    if notice not in previous:path.write_text(notice+previous,encoding='utf-8')
print(json.dumps(dict(status=ready['status'],current_pose_difference_ticks=delta,nominal_motion_seconds=ready['nominal_motion_seconds'],
    command=command,protected_hashes_unchanged=True,assistant_motor_writes=0),ensure_ascii=False))
