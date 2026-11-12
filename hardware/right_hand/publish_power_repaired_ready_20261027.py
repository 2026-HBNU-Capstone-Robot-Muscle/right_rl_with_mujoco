"""Current operator handoff after user confirms V- and responses recover."""
import datetime as dt,json
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,HorizontalQuarterPlan,hashes
from horizontal_quarter_gate_20261027 import validate_review
proof=validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
verification=json.loads((OUT/'verification.json').read_text(encoding='utf-8'))
assert verification['tests_passed']==80 and verification['code_hashes']==hashes()
folder=ROOT/'right_hand_rotation_20261027/vplus_disconnect_211439'
connection=json.loads((folder/'restored_responses_2143.json').read_text(encoding='utf-8'))
connection.update(status='user_confirmed_Vminus_functional_power_restored',wire_terminal_confirmed=True,
    user_confirmed_terminal='V-',user_confirmed_wire_refastened=True,terminal_photo_received=False,
    mechanical_clamping_not_independently_visually_verified=True,next_motion_command_ready=True,
    pending_user_input=None,current_model_review=str(PROOF.relative_to(ROOT)),current_model_cases=210,
    device_free_tests=80,assistant_motor_writes=0)
(folder/'repair_confirmed_review.json').write_text(json.dumps(connection,indent=2)+'\n',encoding='utf-8')
ready=json.loads((OUT/'readiness.json').read_text(encoding='utf-8'))
ready.update(status='Vminus_connection_restored_fresh_pose_ready',timestamp=dt.datetime.now().astimezone().isoformat(),
    latest_connection_review=str((folder/'repair_confirmed_review.json').relative_to(ROOT)),
    latest_off_readonly='repeat_off_readonly_20261027_214344_043175/result.json',
    current_source=str(REQUEST.relative_to(ROOT)),operator_proof=str(PROOF.relative_to(ROOT)),
    latest_camera='rewired_contact_readonly_20261027_214348_618876/result.json',
    latest_source_delta_ticks=0,wire_terminal_user_confirmed='V-',responding_ids=list(range(1,17)),
    all_torque_off_verified=True,voltage_range_v=[5.,5.1],successful_full_reads=100,read_errors=0,
    median_read_ms=connection['median_read_ms'],tests_passed=80,current_empty_static_cases=201,current_empty_dynamic_cases=9,
    current_initial_self_exception_enabled=proof['initial_self_separation']['enabled'],
    held_thumb_book_policy=proof['held_thumb_book_numerical_policy'],
    next_user_command='leap_horizontal_quarter_trial.cmd',next_user_command_ready=True,pending_user_input=None,
    next_user_task='Close Wizard. Keep hand as now. EMPTY firstSPACE; PLACE greenup/orange toward wristID41 then handsaway/SPACE; automaticOPEN then REMOVE/SPACE. Send done only.',
    physical_horizontal_90_verified=False,assistant_motor_writes=0)
(OUT/'readiness.json').write_text(json.dumps(ready,indent=2)+'\n',encoding='utf-8')
(OUT/'operator_guide.md').write_text('''# 전원 재연결 후 현재 자세에서 실행

사용자가 빠졌던 검은 선을 전원장치V-에 고정했다고 확인했다. 이후16개모터 모두응답했고 모두토크OFF,오류0,5.0~5.1V,3Mbps였다. 전체상태100회 읽기오류0,약15ms. 단자사진은 받지 않았으므로 기계적 체결상태를 별도로 시각검증한 것은 아니다.

현재 자세는 `repeat_off_readonly_20261027_214344_043175/result.json` 기준으로 새로 검증했다.201기하+9유한PD조건 모두통과했고,이 자세에서는 초기손가락겹침 예외와 엄지-책0.02mm수치 여유 모두사용하지않았다. 제어/오류차단 테스트80개통과. 원래 v6의20:47자세 요청/검증은 `source_archive_204724`에보관했다.

위자드 및 다른 모터 프로그램을 닫고 `leap_horizontal_quarter_trial.cmd`를한번실행한다. 손가락은현재대로둔다. 빈손첫SPACE,PLACE에서 초록위/주황손목ID41쪽으로큐브를놓고손을뺀뒤두번째SPACE. 큰수평회전과자동OPEN이끝난뒤큐브를빼고세번째SPACE로OFF. 결과는‘했어’만보내면된다.

실제 수평90도는아직검증전이다. 직전실행은 빈손OPEN중43.5초경에전원선이빠진것으로보고됐고,당시모델검사통과후16개통신을읽지못해중단했다. 이문제를bps변경이나전압하한완화로처리하지않았다. 다음실물시험에서큐브회전과전원유지여부를확인한다.
''',encoding='utf-8')
header='''V-재연결 후 재개 준비 완료(2026-10-27):사용자가 검은선을V-에고정했다고 확인.16개모두응답/토크OFF/오류0/5.0~5.1V/3Mbps,100회읽기오류0.현재21:43자세로복귀210조건과테스트80개통과. 이현재자세에서는초기자기접촉예외와엄지-책수치여유모두비활성.기존실제경로/건강기준유지. 다음명령 `leap_horizontal_quarter_trial.cmd` — 빈손첫SPACE→PLACE초록위/주황손목쪽 후SPACE→자동OPEN후큐브제거/SPACE.실제수평90도미검증. 현재근거 `right_hand_rotation_20261027/horizontal_quarter_trial_v6_current/readiness.json`.

<!-- END VMINUS REPAIR READY 20261027 -->

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_text(encoding='utf-8')
    if '<!-- END VMINUS REPAIR READY 20261027 -->' not in old:path.write_text(header+old,encoding='utf-8')
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
             'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name
    if path.exists():
        data=json.loads(path.read_text(encoding='utf-8'));data.update(current_status=ready['status'],
            current_hand_review=str((OUT/'readiness.json').relative_to(ROOT)),next_user_command=ready['next_user_command'],
            next_user_command_ready=True,pending_user_input=None)
        path.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(status=ready['status'],model_cases=210,tests=80,motor_writes=0,next_user_command=ready['next_user_command']),ensure_ascii=False))
