"""Publish the measured load question and the unchanged-motion EMPTY comparison."""
import datetime,hashlib,json
import numpy as np
from yaw_load_empty_plan_20261029 import ROOT,OUT,REQUEST,PROOF,YawLoadEmptyPlan
from yaw_load_empty_gate_20261029 import validate_review
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
p=YawLoadEmptyPlan(REQUEST);proof=validate_review(p,PROOF)
actual_path=ROOT/'four_reply_yaw_20261029_052443_385204/offline_review/grip_load_review.json';actual=read(actual_path)
fresh_path=ROOT/'fast_off_readonly_20261029_054330_536705/result.json';fresh=read(fresh_path)
assert fresh['status']=='off_snapshots_saved' and fresh['maximum_drift_ticks']<=2 and not fresh['servo_writes'] and not fresh['torque_changes']
delta=int(np.max(abs(np.asarray([s['position'] for s in fresh['snapshots']])-p.source_position())))
assert delta<=4,'Current pose requires a new model review'
for name,digest in {'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
    'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
    'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
    'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d'}.items():assert sha(ROOT/name)==digest,name
scout_path=ROOT/'right_hand_rotation_20261029/thumb_relief_scout/review.json';scout=read(scout_path)
baseline=next(c for c in scout['cases'] if c['candidate']=='ID1_+0');relief=next(c for c in scout['cases'] if c['candidate']=='ID2_-16')
scout_summary=dict(status='nominal_model_hypothesis_only',source=scout_path.relative_to(ROOT).as_posix(),
    tested_candidates=len(scout['cases']),all_candidates_nominally_passed=all(c['passed'] for c in scout['cases']),
    thumb_motor2_delta_ticks=-16,thumb_motor2_delta_degrees=-1.40625,
    model_normal_force_relative_change=relief['mean_model_total_cube_normal_force']/baseline['mean_model_total_cube_normal_force']-1,
    model_middle_root_torque_relative_change=relief['mean_model_middle_root_torque']/baseline['mean_model_middle_root_torque']-1,
    actual_force_not_calibrated=True,not_an_operator_target=True,hardware_candidate_ready=False,not_used_in_EMPTY_comparison=True)
now=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat()
ready=dict(status='EMPTY_same_yaw_load_comparison_ready',timestamp=now,proof=PROOF.relative_to(ROOT).as_posix(),proof_sha256=sha(PROOF),
    latest_readonly=fresh_path.relative_to(ROOT).as_posix(),current_pose_difference_ticks=delta,current_recovery_cases=210,
    complete_hand_prefix_dynamics_cases=9,regression_checks_passed=8,test_source_sha256=sha(ROOT/'test_yaw_load_empty_20261029.py'),
    same_original_joint_targets=True,yaw_targets=320,previous_loaded_stop_yaw_samples=267,nominal_motion_seconds=proof['nominal_motion_seconds'],
    active_motion_limit_s=300,final_fixed_hold_seconds=1.5,cube_stays_OUT=True,one_terminal_SPACE_only=True,no_PLACE_or_REMOVE=True,
    current_caps_PID_health_stops_unchanged=True,goal_current450_is_current_limit_not_contact_force=True,
    measured_cube_force_sensor_available=False,cause_not_identified=True,latest_loaded_evidence=actual,
    nominal_thumb_scout=scout_summary,assistant_motor_writes=0)
(OUT/'readiness.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
guide='''같은 관절 동작의 빈손 부하 비교

큐브는 처음부터 끝까지 손과 카메라 시야 밖에 둡니다. 손가락과 받침은 현재 그대로 두세요.
leap_yaw_load_empty.cmd를 실행하고 터미널에서 SPACE 한 번만 누릅니다.
준비 자세, 같은 잡기 동작, 같은 회전 관절 구간320개를 빈손으로 움직이고1.5초 고정한 뒤 자동 OFF합니다.
약2분20초이며 PLACE나 REMOVE, 추가 SPACE는 없습니다. Q/Esc/Ctrl+C는 중단합니다.
끝나거나 멈추면 했어라고 알려주면 됩니다. 로그 전체를 붙일 필요는 없습니다.

직전은 통신 누락0건, 최저4.9V, 검정 카메라 프레임0개였고 회전 중ID12(중지 뿌리)가119틱(약10.5도)
뒤처지며 중단됐습니다. 그때209mA였고 이전 완료 실행의 이 관절 최대 전류는109mA였습니다.
토크OFF/원래 설정 복구와 사용자 큐브 제거를 확인했습니다.
현재모드5에서Goal Current450mA는전류상한이며 큐브에 가해지는 접촉력을 직접 측정한 값이 아닙니다.
공식 설명: https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#goal-current102

빈손에서도 같은 부하/지연이 생기면 기구, 다른 손가락, 케이블 또는 구동 응답을 우선 확인합니다.
빈손에서는 잘 따라가면 큐브 접촉, 받침, 파지 간격 쪽을 우선 조정할 근거가 됩니다.
이 결과만으로 특정 접촉 위치나 힘을 확정하지 않습니다.
현재 명령은 힘/PID/전압/온도/추종/재시도 기준을 바꾸지 않았고, 이전267번째에서 멈춘 동일 경로를320번째까지 비교합니다.
현재 복귀201정적+9동적 및 빈손 전체 비교 구간9동적 조건을 통과했습니다.
엄지 각도 후보는별도모델계산에만있으며 이번 실물 명령에는 적용하지 않았습니다.
'''
(OUT/'실행안내.md').write_text(guide,encoding='utf-8')
command='& "'+str(ROOT/'leap_yaw_load_empty.cmd')+'"'
for filename in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/filename;r=read(path)
    r.update(status=ready['status'],current_status=ready['status'],timestamp=now,
        next_command=command,next_user_command=command,next_user_command_ready=True,next_physical_command_ready=True,
        next_user_action_required=True,next_user_action='EMPTY throughout: one terminal SPACE, same grasp/yaw320, automatic OFF; no cube placement',
        next_physical_command_kind='EMPTY_SAME_YAW_LOAD_COMPARISON',next_step='Separate cube-contact load from mechanical/cable/actuator resistance',
        trajectory='EMPTY preparation -> same grasp -> same yaw prefix320 -> hold1.5s -> OFF',cube_orientation_at_place=None,next_user_orientation=None,
        nominal_motion_seconds=ready['nominal_motion_seconds'],active_motion_limit_s=300,
        source_off_readonly=ready['latest_readonly'],latest_readonly=ready['latest_readonly'],latest_pose_difference_from_review_ticks=delta,
        proof=ready['proof'],proof_sha256=ready['proof_sha256'],software_checks_passed=8,
        latest_loaded_grip_evidence=actual,next_yaw_readiness=ready,current_cube_removed_confirmed_by_user=True,
        latest_reference_speed_status='four-reply trial reached yaw but stopped on ID12 tracking; not a communication stop',
        yaw_motion_ready=False,empty_load_comparison_ready=True,motors_off_verified=True,original_settings_restored_now=True,
        physical_combined_rotation_verified=False,assistant_motor_writes=0,assistant_motor_or_setting_writes=0)
    path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
notice=('2026-10-29 05:43 현재 단계: 중지 부하를 분리하기 위한 동일 경로 빈손 비교 준비. '
    'four_reply_yaw_20261029_052443_385204은 회전267개 상태까지 진행했고ID12 목표2322/실제2203,119틱(10.459도),209mA에서 중단됐다. '
    '통신 누락0건, 최저4.9V, 카메라1171프레임/검정0개였다. OFF/원래 설정 복구와 사용자 큐브 제거를 확인했다. '
    '같은관절 이전 완료 기록 최대109mA와 비교하면 부하가 커졌지만, 전체 파지력 과다나 특정 접촉이 원인이라고 확정할 수 없다. '
    '공식XC330 모드5의Goal Current는전류상한이고실제큐브접촉력센서가아니다. '
    '엄지±1.4도후보5개를nominal 모델에서계산해,ID2 -16틱 후보는 모델평균법선힘약20%/중지뿌리토크약7% 감소였으나 '
    '실제힘이보정되지않았고 넓은조건검증/실물적용은아직하지않았다. '
    '다음 leap_yaw_load_empty.cmd는 이전과정확히같은잡기+회전320개목표를큐브없이수행한다. '
    '현재복귀201정적+9동적,전체빈손구간9동적모두통과(최대추종0.05631rad). '
    '같은힘/PID/건강기준과4개씩전체읽기를유지하고 SPACE한번,PLACE없음,최종1.5초고정후자동OFF한다. '
    '8개검증테스트통과.현재읽기와모델출발점차이최대'+str(delta)+'틱,명목134.775초/기존300초제한. '
    '빈손비교실물결과는아직없다.\n\n')
for filename in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/filename;previous=path.read_text(encoding='utf-8')
    if notice not in previous:path.write_text(notice+previous,encoding='utf-8')
print(json.dumps(dict(status=ready['status'],current_pose_difference_ticks=delta,nominal_motion_seconds=ready['nominal_motion_seconds'],command=command,assistant_motor_writes=0),ensure_ascii=False))
