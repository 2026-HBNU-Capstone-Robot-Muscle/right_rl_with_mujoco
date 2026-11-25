"""Publish the completed empty observation and a concrete next operator step."""
import hashlib,json,datetime
import numpy as np
from paced_retained_yaw_plan_20261029 import ROOT,OUT,REQUEST,PROOF,PacedRetainedYawPlan
from paced_retained_yaw_gate_20261029 import validate_review
from review_empty_hold_comm_20261029 import review as hold_review
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
p=PacedRetainedYawPlan(REQUEST);r=validate_review(p,PROOF);hold=hold_review()
fresh_path=ROOT/'fast_off_readonly_20261029_043104_757807/result.json';fresh=read(fresh_path)
assert fresh['status']=='off_snapshots_saved' and fresh['maximum_drift_ticks']<=2 and not fresh['servo_writes'] and not fresh['torque_changes']
difference=int(np.max(abs(np.asarray([s['position'] for s in fresh['snapshots']])-p.source_position())))
assert difference<=4,'Fresh current pose requires a new model check'
for name,digest in {'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
    'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
    'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
    'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d'}.items():assert sha(ROOT/name)==digest,name
ready=dict(status='supervised_paced_retained_yaw_ready',timestamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat(),
    proof=PROOF.relative_to(ROOT).as_posix(),proof_sha256=sha(PROOF),empty_model_cases=210,
    regression_checks_passed=11,test_source_sha256=sha(ROOT/'test_paced_retained_yaw_20261029.py'),
    completed_empty_hold=hold,fresh_readonly=fresh_path.relative_to(ROOT).as_posix(),fresh_readonly_sha256=sha(fresh_path),
    maximum_current_pose_difference_ticks=difference,nominal_motion_seconds=r['nominal_motion_seconds'],active_motion_limit_s=300,
    original_health_and_goal_checks_retained=True,loaded_targets_unchanged=True,motion_sample_period_s=.075,
    passive_observation_period_s=.075,distinct_yaw_target_period_s=.15,settings_write_retries=0,
    communication_stability_not_certified=True,physical_rotation_not_yet_observed=True,assistant_motor_writes=0)
(OUT/'readiness.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
command='& "'+str(ROOT/'leap_paced_retained_yaw.cmd')+'"'
guide='''수평 회전 시험

1. 큐브를 손과 카메라 시야 밖에 둡니다. 손가락과 받침은 현재 상태 그대로 두세요.
2. leap_paced_retained_yaw.cmd를 실행하고 터미널에서 첫 SPACE를 누릅니다.
3. PLACE CUBE가 뜬 뒤에만 큐브를 놓습니다. 빨강 면 중앙ID2가 위, 초록 면 중앙ID5가 손목ID41 쪽입니다.
   위치 예시: ../../retained_slow_yaw_20261029_032447_137236/cube_before_grasp.jpg
   큐브가 스스로 놓이면 손을 빼고 두 번째 SPACE를 누릅니다.
4. 느린 잡기와 수평 회전, 자동 OPEN을 기다립니다. 동작은 약4~5분이며 배치 시간은 별도입니다.
5. REMOVE가 뜨면 큐브를 빼고 세 번째 SPACE를 누릅니다. OFF와 설정 복구가 기록됩니다.
Q/Esc/Ctrl+C는 중단합니다. 끝나거나 멈추면 했어라고 알려주세요. 로그 전체를 붙일 필요는 없습니다.

45초 빈손 유지 검사는595회 상태 확인을 마쳤고, 토크OFF와 원래 설정 복구가 확인됐습니다.
부분 수신7건은 정상 상태를 두 번 다시 읽고 회복했습니다. 통신 문제가 완전히 해결됐다는 뜻은 아닙니다.
이번 파일은 배치 대기/사진/카운트다운/제거 대기의 읽기도75ms 주기로 맞췄고, OFF 설정 확인은
완료된 빈손 검사와 같은 전체 고속 읽기를 사용합니다. 기존 수평 회전 목표와 속도, 전류/PID,
전압/온도/추종/200ms 정상 상태2회 확인은 유지합니다. 실제 회전량은 새 저장 영상으로 확인합니다.
'''
(OUT/'실행안내.md').write_text(guide,encoding='utf-8')
archive_keys=('trajectory','cube_orientation_at_place','nominal_motion_seconds','active_motion_limit_s','proof','proof_sha256','maximum_aggregate_thumb_displacement_ticks')
rows=p.plan(p.source_position());thumb_max=int(np.max(abs(np.asarray([q for _,q in rows])[:,:4]-p.source_position()[:4])))
for filename in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/filename;old=read(path)
    if 'previous_combined_flat_summary' not in old:old['previous_combined_flat_summary']={k:old[k] for k in archive_keys if k in old}
    old.update(status=ready['status'],current_status=ready['status'],timestamp=ready['timestamp'],
        next_command=command,next_user_command=command,next_user_command_ready=True,next_physical_command_ready=True,
        next_user_action_required=True,next_user_action='Run paced retained-yaw once: EMPTY first SPACE; PLACE RED up/GREEN wristward second SPACE; REMOVE third SPACE',
        next_physical_command_kind='SUPERVISED_RED_UP_SLOW_YAW',
        next_step='EMPTY preparation -> PLACE RED up/GREEN wristward -> grasp -> slow yaw -> OPEN -> REMOVE -> OFF',
        trajectory='EMPTY preparation -> PLACE RED up/GREEN wristward -> grasp -> slow yaw -> OPEN -> REMOVE -> OFF',
        cube_orientation_at_place='RED up; GREEN toward WRIST/ID41',cube_reposition_between_rotations_required=False,
        nominal_motion_seconds=r['nominal_motion_seconds'],active_motion_limit_s=300,maximum_aggregate_thumb_displacement_ticks=thumb_max,
        source_off_readonly=fresh_path.relative_to(ROOT).as_posix(),source_off_drift_ticks=fresh['maximum_drift_ticks'],
        latest_readonly=fresh_path.relative_to(ROOT).as_posix(),latest_pose_difference_from_review_ticks=difference,latest_pose_cache_verified=True,
        proof=ready['proof'],proof_sha256=ready['proof_sha256'],software_checks_passed=11,
        P_restoration_pending=False,original_settings_restored_now=True,motors_off_verified=True,
        empty_hold_complete=True,empty_hold_result=hold,empty_hold_readiness={'status':'completed','next_command':None},
        next_yaw_readiness=ready,yaw_motion_ready=True,physical_combined_rotation_verified=False,
        assistant_motor_or_setting_writes=0,assistant_motor_writes=0)
    path.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
notice=('2026-10-29 04:31 현재 단계: 빈손45초 유지 검사 완료, 다음은 수평 회전 시험. '
    'empty_hold_comm_20261029_042239_202853은 준비 이동과45초595회 관찰을 마쳤다. '
    '최저4.9V, 최대124mA, 최대 추종0.10431rad이며 카메라732프레임 모두 영상 신호가 있었다. '
    '부분 읽기7건은5회 회복 절차로 정상 전체 상태2회 확인됐고 최대172.035ms였다. '
    '토크OFF와P2100/D1900 등 원래 설정 복구가 확인됐다. 통신 원인은 아직 확정되지 않았다. '
    'OFF 직후 손가락이 내려와 있던 마지막 두 자세는 출발점으로 재사용하지 않고,04:25 안정된 각도를 새로 읽어 '
    '현재 복귀 경로201개 정적 및9개 동적 조건을 검사했다.04:31 재확인 차이는 최대'+str(difference)+'틱이다. '
    'leap_paced_retained_yaw.cmd는 기존 수평 경로를 그대로 사용하며 배치/사진/카운트다운/제거 대기를75ms 주기로 읽는다. '
    'OFF 설정 확인은 이번 완료된 진단의 EmptyBus 구현을 그대로 상속한다. '
    '취소/건강 검사 전파, 주기, 원래 관절 감시, 경로 동일성, 증거 변조 거부 및 기본 무장치 동작11개 테스트 통과. '
    '명목 동작241.425초/기존300초 제한이며, PLACE에서 빨강 위/초록 손목ID41 쪽이다. '
    '이 실행은 아직 하지 않았고, 실제90도 및 넓은 조건의 성공은 확인되지 않았다. 기존3/27 전체 모델 결과는 그대로 보존한다.\n\n')
for filename in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/filename;text=path.read_text(encoding='utf-8')
    if notice not in text:path.write_text(notice+text,encoding='utf-8')
print(json.dumps(dict(status=ready['status'],current_pose_difference_ticks=difference,nominal_motion_s=ready['nominal_motion_seconds'],command=command,assistant_motor_writes=0),ensure_ascii=False))
