"""Publish verified original restoration and the next short EMPTY experiment."""
import hashlib,json
import numpy as np
from empty_hold_comm_plan_20261029 import ROOT,OUT,REQUEST,PROOF,EmptyHoldCommPlan,hashes
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
repair_path=ROOT/'restore_off_P_20261029_040625_534298/result.json';repair=read(repair_path)
assert repair['status']=='original_P_restored_OFF_verified' and repair['torque_changes']==0 and repair['goal_writes']==0
for snapshot in repair['snapshots'][-2:]:
    assert not any(snapshot['state']['torque']) and snapshot['state']['p_gain']==[2100]*16
p=EmptyHoldCommPlan(REQUEST);proof=read(PROOF);assert proof['code_hashes']==hashes()
fresh_path=ROOT/'fast_off_readonly_20261029_041945_791784/result.json';fresh=read(fresh_path)
assert fresh['status']=='off_snapshots_saved' and fresh['maximum_drift_ticks']<=2 and fresh['servo_writes']==0 and fresh['torque_changes']==0
difference=np.asarray([s['position'] for s in fresh['snapshots']])-p.source_position()
assert int(np.max(abs(difference)))<=4,'Current pose requires a fresh model screen'
ready=dict(status='user_started_empty_hold_comm_ready',cube_stays_OUT=True,one_SPACE_only=True,
    preparation_goals=len(p.plan(p.source_position())),fixed_hold_seconds=45.,hold_sampling_period_ms=75,
    nominal_motion_and_hold_seconds=proof['nominal_seconds'],current_reference_maximum_difference_ticks=int(np.max(abs(difference))),
    fresh_reference=fresh_path.relative_to(ROOT).as_posix(),fresh_reference_sha256=sha(fresh_path),
    model_cases_passed=210,extended_fixed_hold_dynamics_verified=True,original_setting_restoration_verified=True,
    restoration_evidence=repair_path.relative_to(ROOT).as_posix(),restoration_evidence_sha256=sha(repair_path),
    plan_review_sha256=sha(PROOF),tests_passed=11,
    test_source_sha256=sha(ROOT/'test_empty_hold_comm_20261029.py'),
    hardware_health_limits_unchanged=True,original_held_joint_guard_retained=True,
    no_goal_writes_during45s_probe=True,OFF_setting_readback_via_complete_fast_snapshot=True,
    settings_write_retries=0,communication_root_cause_not_yet_verified=True,assistant_motor_writes=0)
(OUT/'readiness.json').write_text(json.dumps(ready,indent=2)+'\n',encoding='utf-8')
guide='''빈손 정지 유지 통신 검사

큐브는 처음부터 끝까지 손과 카메라 시야 밖에 둡니다. 손가락과 받침은 현재 상태로 두세요.
로봇 USB와 기존5V 전원, Streaming Webcam을 연결한 상태에서 leap_empty_hold_comm.cmd를 실행합니다.
터미널에서 SPACE를 한 번 누르면 빈손 준비 자세로 이동하고,45초 동안 같은 목표를 유지한 뒤 자동 OFF합니다.
약1분30초이며 큐브를 놓는 단계나 추가 SPACE는 없습니다. Q/Esc/Ctrl+C는 중단합니다.

목적: 실제 이동과 같은75ms 상태 확인 주기로, 큐브 접촉 없이 구동 중 통신을 관찰합니다.
정지 유지45초 중에는 새로운 목표를 보내지 않습니다. 원래 전류/PID/전압/온도/추종/held-joint 검사와
200ms 안의 정상 전체 상태2회 확인을 유지합니다.
OFF 뒤 원래 설정 확인에는 전체 고속 상태 읽기를 사용합니다. 설정 쓰기 재시도는 없습니다.
통신 문제가 해결됐다고 판정한 단계는 아니며, 이번 기록으로 다음 조정을 결정합니다.
끝나면 했어라고 알려주면 저장 파일을 확인합니다.
'''
(OUT/'실행안내.md').write_text(guide,encoding='utf-8')
command='& "'+str(ROOT/'leap_empty_hold_comm.cmd')+'"'
for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name;r=read(path);r.update(status='ready_empty_hold_comm_after_original_P_restored',
        current_status='ready_empty_hold_comm_after_original_P_restored',next_command=command,next_physical_command_ready=True,
        next_physical_command_kind='EMPTY_PREPARATION_AND45s_FIXED_HOLD',P_restoration_pending=False,original_settings_restored_now=True,
        restoration_evidence=ready['restoration_evidence'],empty_hold_readiness=ready,
        next_step='Cube OUT throughout, one SPACE, preparation then45s fixed-goal health reads at75ms, automatic OFF',
        yaw_motion_ready=False,assistant_motor_writes=0)
    path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
notice=('2026-10-29 04:06 사용자 실행으로 원래P2100 복구 완료. 설정84주소의 원래값 한 번 쓰기 후 두 OFF 상태에서 확인됐고 '
    '목표/토크/EEPROM 쓰기는 없었다. 다음은 큐브 없는 짧은 통신 유지 진단이다. '
    '현재 준비 경로540개 목표와 최종45초 유지의210개 모델 조건이 통과했다. 실제 최근 읽기와 출발점 차이는 최대'
    +str(ready['current_reference_maximum_difference_ticks'])+'틱이다. '
    'leap_empty_hold_comm.cmd 실행 후 SPACE 한 번으로 준비 자세 이동과45초 고정 상태 관찰을 수행하고 자동 OFF한다. '
    '명목85.5초, 유지 상태 읽기는75ms 주기로 실제 이동 주기와 같으며 이 구간 목표 쓰기는 없다. '
    '큐브 확인을 가짜로 통과시키는 절차는 없고, 원래 runner의 held-joint guard를 그대로 사용하도록 객체만 노출했다. '
    'runner 제어 흐름은 AST 비교로 같음을 확인했고 실행/관절 감시/주기/기본 무장치 검사11개를 통과했다. '
    'OFF 설정 복구 확인은 원래 RAM 주소를 전체 고속 스냅샷에서 읽고 토크 OFF와 오류0도 확인하도록 바꿨다. '
    '목표 확인은 이 경로로 보내지 않고 설정 쓰기를 재시도하지 않는다. 통신 원인과 수평 회전 완료는 아직 확인되지 않았다.\n\n')
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_text(encoding='utf-8')
    if notice not in old:path.write_text(notice+old,encoding='utf-8')
for name,digest in {'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
    'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
    'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
    'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d'}.items():assert sha(ROOT/name)==digest,name
print(json.dumps(dict(status=ready['status'],command=command,nominal_seconds=ready['nominal_motion_and_hold_seconds'],
    current_reference_maximum_difference_ticks=ready['current_reference_maximum_difference_ticks'],assistant_motor_writes=0),ensure_ascii=False))
