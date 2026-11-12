"""Preserve the partial-turn result and publish the checked baseline reproduction."""
import datetime,hashlib,json
import numpy as np
from reference_speed_yaw_plan_20261029 import ROOT,OUT,REQUEST,PROOF,ReferenceSpeedYawPlan
from reference_speed_yaw_gate_20261029 import validate_review
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
p=ReferenceSpeedYawPlan(REQUEST);proof=validate_review(p,PROOF)
folder=ROOT/'paced_retained_yaw_20261029_044827_659675';stop=read(folder/'offline_review/stop_review.json')
trace=read(folder/'offline_review/side_marker_rotation_trace.json')
stop.update(cube_removal_confirmed_by_user=True,cube_currently_present=False,
    user_confirmation='request_user_input_async reply: cube removed',
    provisional_final_camera_relative_angle_deg=trace['final_median_angle_deg'],
    last_one_second_valid_marker_frames=trace['final_window_accepted_frames'],
    metric_robot_axis_angle_certified=False)
(folder/'offline_review/stop_review.json').write_text(json.dumps(stop,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
fresh_path=ROOT/'fast_off_readonly_20261029_050438_127194/result.json';fresh=read(fresh_path)
assert fresh['status']=='off_snapshots_saved' and fresh['maximum_drift_ticks']<=2 and not fresh['servo_writes'] and not fresh['torque_changes']
delta=int(np.max(abs(np.asarray([s['position'] for s in fresh['snapshots']])-p.source_position())))
assert delta<=4,'Current pose requires a new model check'
for name,digest in {'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
    'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
    'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
    'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d'}.items():assert sha(ROOT/name)==digest,name
now=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat()
ready=dict(status='reference_speed_yaw_supervised_reproduction_ready',timestamp=now,
    proof=PROOF.relative_to(ROOT).as_posix(),proof_sha256=sha(PROOF),source_readonly=fresh_path.relative_to(ROOT).as_posix(),
    current_pose_difference_ticks=delta,empty_recovery_cases=210,
    original_loaded_contact_and_tracking_model_cases=27,original_loaded_geometry_cases=201,
    original_model_nominal_angle_deg=proof['loaded_model_nominal_angle_deg'],original_model_does_not_certify90deg=True,
    completed_physical_reference=proof['completed_physical_reference'],
    regression_checks_passed=11,test_source_sha256=sha(ROOT/'test_reference_speed_yaw_20261029.py'),
    loaded_goal_vectors_and_order_match_completed_physical_writes=True,nominal_motion_seconds=proof['nominal_motion_seconds'],
    initial_orientation='GREEN up; ORANGE toward WRIST/ID41',yaw_target_period_s=.075,
    previous_failed_slow_yaw_target_period_s=.15,
    placement_and_speed_changed_together_for_baseline_reproduction=True,cause_not_identified=True,
    gains_current_caps_health_thresholds_unchanged=True,automatic_goal_or_setting_resend=False,
    latest_failed_trial=stop,assistant_motor_writes=0,physical_success_of_new_run_not_verified=True)
(OUT/'readiness.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
guide='''이전 완료 조건 재현

1. 큐브는 손과 카메라 시야 밖에 둡니다. 손가락과 받침은 현재 그대로 둡니다.
2. leap_reference_speed_yaw.cmd 실행 후 터미널에서 첫 SPACE를 누릅니다.
3. PLACE CUBE가 뜬 뒤에만 초록 면이 위, 주황 면이 손목ID41 쪽이 되게 놓습니다.
   배치 예시: ../../calibrated_quarter_trial_20261028_174723_390729/cube_before_grasp.jpg
   스스로 놓이는 위치에서 손을 빼고 두 번째 SPACE를 누릅니다.
4. 수평 회전과 자동 OPEN을 기다립니다. 명목 동작은 약3분36초이며 배치 시간은 별도입니다.
5. REMOVE가 뜨면 큐브를 빼고 세 번째 SPACE를 눌러 OFF와 설정 복구를 확인합니다.
Q/Esc/Ctrl+C로 중단합니다. 완료되거나 멈추면 했어라고 알려주면 됩니다.

직전 실행은 중지 뿌리ID12가118틱(약10.4도) 뒤처져 중단됐습니다. 전압은4.9V 이상이었고
카메라1287프레임에 검정 프레임이 없었으며 통신 실패2건도 회복됐습니다. OFF와 설정 복구가 확인됐습니다.
초록ID5 영상 추정은 마지막 구간 약19~22도이며 마지막1초에는 유효 추정1개뿐입니다.
이를 정확한 로봇 축 회전각으로 인증하지 않습니다. 큐브는 사용자 확인으로 제거됐습니다.

직전은 서로 다른 회전 목표 사이가150ms였고, 이전 완료 실행은75ms였습니다.
새 파일은 이전 완료 실행의 실제2032개 loaded 목표 쓰기 전체와 순서, 명목75ms 주기와
초록 위/주황 손목 방향을 재현합니다. 속도와 방향을 함께 이전 조건으로 맞추는 비교이므로
지연의 원인을 속도나 배치 중 하나로 확정한 것은 아닙니다.
관절 힘과 중단 기준을 높이지 않고, 배치 대기75ms와 OFF 고속 설정 확인은 유지합니다.
기존 접촉/추종 모델27조건은 통과했지만 모델 명목 회전은 약62도이며90도 보증은 아닙니다.
실제 각도는 새 영상으로 확인합니다. 이번 새 실행은 아직 하지 않았습니다.
'''
(OUT/'실행안내.md').write_text(guide,encoding='utf-8')
command='& "'+str(ROOT/'leap_reference_speed_yaw.cmd')+'"'
for filename in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/filename;r=read(path)
    if 'previous_paced_red_yaw_proposal' not in r:r['previous_paced_red_yaw_proposal']={k:r[k] for k in ('trajectory','cube_orientation_at_place','proof','proof_sha256','nominal_motion_seconds') if k in r}
    r.update(status=ready['status'],current_status=ready['status'],timestamp=now,
        next_command=command,next_user_command=command,next_user_command_ready=True,next_physical_command_ready=True,
        next_user_action_required=True,next_user_action='EMPTY first SPACE; PLACE GREEN up/ORANGE wristward second SPACE; REMOVE third SPACE',
        next_step='Reproduce completed physical nominal75ms yaw and GREEN-up placement',
        next_physical_command_kind='REFERENCE_SPEED_GREEN_UP_YAW',trajectory='EMPTY -> PLACE GREEN up/ORANGE wristward -> original75ms yaw -> OPEN -> REMOVE -> OFF',
        cube_orientation_at_place=ready['initial_orientation'],nominal_motion_seconds=ready['nominal_motion_seconds'],
        active_motion_limit_s=300,source_off_readonly=ready['source_readonly'],latest_readonly=ready['source_readonly'],
        latest_pose_difference_from_review_ticks=delta,latest_pose_cache_verified=True,
        proof=ready['proof'],proof_sha256=ready['proof_sha256'],software_checks_passed=11,
        yaw_distinct_goal_period_s=.075,ordinary_cycle_s=.075,motors_off_verified=True,original_settings_restored_now=True,
        latest_actual_yaw_trial=stop,current_cube_removed_confirmed_by_user=True,next_yaw_readiness=ready,
        nominal_model_yaw_current_reference_deg=proof['loaded_model_nominal_angle_deg'],
        current_model_contact_tracking_cases_passed=27,current_model90deg_certified=False,
        next_user_orientation='GREEN up; ORANGE wristward',yaw_motion_ready=True,physical_combined_rotation_verified=False,
        assistant_motor_or_setting_writes=0,assistant_motor_writes=0)
    path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
notice=('2026-10-29 05:04 현재 단계: 이전에 완료된 회전 속도와 배치 재현 준비. '
    '직전 paced_retained_yaw_20261029_044827_659675은 수평 구간510개 상태까지 진행했지만 '
    '중지 뿌리ID12 목표2301/실제2183(118틱,0.181rad),207mA에서 중단됐다. '
    '최저4.9V, 카메라1287프레임 모두 정상 신호, 통신 부분 수신2건은 회복됐다. '
    'OFF/원래 설정 복구와 사용자 큐브 제거가 확인됐다. 초록ID5 상대 회전 마지막 유효8개의 중앙값18.67도는10초 구간을 포함했고, '
    '마지막1초 유효1개는22.18도였다. 약20도 안팎의 부분 회전이며 로봇 축 각도 인증은 아니다. '
    '동일 목표에서 성공 기록ID12 전류77mA/지연44틱에 비해 이번199mA/113틱이었다. '
    '실패 실행은 지령150ms/빨강 위였고 이전 완료 기록은75ms/초록 위였다. '
    '새 leap_reference_speed_yaw.cmd는 완료 기록의 실제2032개 loaded 목표와 순서를 그대로 재현한다. '
    '힘/중단 기준은 유지하고75ms 배치 대기와 OFF 고속 설정 확인을 상속한다. '
    '현재 빈손 복귀201정적+9동적 조건 통과, 기존2032개 loaded 접촉/추종27조건 및 정적201조건 확인, '
    '새 버전11개 테스트 통과. 모델 명목 회전은약62도이고 실제 새 실행 성공은 아직 미확인이다. '
    '현재 읽기와 모델 출발점 차이최대'+str(delta)+'틱, 명목215.925초/기존300초 제한. '
    '속도와 시작 방향을 함께 재현하므로 원인을 둘 중 하나로 확정하지 않는다.\n\n')
for filename in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/filename;previous=path.read_text(encoding='utf-8')
    if notice not in previous:path.write_text(notice+previous,encoding='utf-8')
print(json.dumps(dict(status=ready['status'],current_pose_difference_ticks=delta,nominal_motion_s=ready['nominal_motion_seconds'],command=command,assistant_motor_writes=0),ensure_ascii=False))
