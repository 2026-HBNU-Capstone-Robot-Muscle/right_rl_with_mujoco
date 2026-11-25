"""Retire the pre-enable failures and publish the bounded OFF-only fix."""
import datetime as dt,hashlib,json
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,LOADED_PATH,HorizontalQuarterPlan
from horizontal_quarter_gate_20261027 import validate_review
from horizontal_preload_confirmation_20261027 import POLICY

def main():
    plan=HorizontalQuarterPlan(REQUEST);proof=validate_review(plan,PROOF);previous=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v1';baseline=json.loads((previous/'review.json').read_bytes());relative=LOADED_PATH.relative_to(ROOT).as_posix();assert baseline['code_hashes'][relative]==hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest()
    failures=[]
    for name in ('horizontal_quarter_trial_20261027_141354_998533','horizontal_quarter_trial_20261027_141539_818724'):
        r=json.loads((ROOT/name/'result.json').read_bytes());assert r['error']=='OFF preload unstable; no enable' and r['all_torque_off_verified'] and r['settings_restored'];assert not r.get('samples') and not r.get('enable_snapshots');failures.append(dict(folder=name,error=r['error'],motion_started=False,all_torque_off_verified=True,settings_restored=True))
    shared=ROOT/'camera_gap_motion_runner_20261026.py';assert hashlib.sha256(shared.read_bytes()).hexdigest()=='7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d'
    state=json.loads((previous/'readiness.json').read_bytes());state.update(status='horizontal_quarter_v2_preload_fix_ready',timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat(),operator_model_review=PROOF.relative_to(ROOT).as_posix(),latest_readonly='repeat_off_readonly_20261027_142049_227895/result.json',latest_readonly_stable_drift_ticks=2,latest_user_trial=failures[-1]['folder'],recent_operator_stops=failures,off_preload_confirmation_policy=dict(POLICY),device_free_tests_passed=42,motion_controls_identical_to_v1=True,loaded_path_sha256=hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest(),shared_motion_runner_unchanged=True,next_user_command='leap_horizontal_quarter_trial.cmd',next_user_command_ready=True,next_user_task='Run the same command once, confirm v2 header. EMPTY first SPACE; PLACE then second SPACE; REMOVE after automatic opening then third SPACE. Send done only.',next_user_action='operator_starts_v2_once',next_motion_command_ready=True,rotation_execution_ready=True,physical_horizontal_90_verified=False,horizontal_quarter_turn_verified=False,assistant_motor_writes=0,assistant_torque_or_setting_writes=0,pending_user_input=None)
    (OUT/'readiness.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');(OUT/'preload_fix_review.json').write_text(json.dumps(dict(failures=failures,policy=POLICY,tests_passed=42,current_empty_model_cases=210,unchanged_loaded_static_cases=201,unchanged_loaded_physics_cases=27,all_code_from_enable_through_motion_and_cleanup_unchanged=True,motion_data_unchanged=True,assistant_motor_writes=0),indent=2)+'\n',encoding='utf-8')
    guide='''# 수평 회전 v2 시작 검사 수정

14:13과14:15 실행은 모두 토크를 켜기 전에 ID10의2틱 변화를1틱 안정성 검사에서 거부했다. 회전 동작은 시작하지 않았고 설정 복구와 OFF가 확인됐다.

v2는 OFF 상태에서250ms 시간 예산 안에 새 읽기3회의 현재값과 목표값 범위가2틱 이내인지 확인한다. 이는 앞단의 기존 정지 읽기 기준과 같다. 원래 자세와 목표의4틱 한도, 전원/온도/오류/설정 검사는 매번 유지한다. 목표나 설정을 다시 쓰지 않는다. 토크를 켠 뒤의 안정성 검사와 실제 이동 및 종료 코드는 그대로다.

장치 없는42개 검사와 새 현재 자세의 복귀210조건이 통과했다. 회전 경로와 기존 loaded 기하201조건, 물리27조건은 값과 해시가 같다. 최종 모델 영상은 ../horizontal_quarter_trial_v1/model_video/model_yaw_diagnostic_3x.mp4 이다. 실물90도는 아직 확인 전이다.

같은 leap_horizontal_quarter_trial.cmd 를 한 번 실행해 헤더 HORIZONTAL QUARTER-TURN v2 를 확인한다. 큐브는 손과 카메라 밖에 두고 첫 SPACE. PLACE가 나온 뒤 초록 위, 주황 손목ID41 쪽으로 놓고 사람 손을 뺀 뒤 두 번째 SPACE. 회전과 자동 열기가 끝나면 큐브를 빼고 세 번째 SPACE로 OFF한다. 끝나거나 중단되면 했어만 보내면 된다.
'''
    (OUT/'operator_guide.md').write_text(guide,encoding='utf-8')
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261026/fresh_empty_start_20261026_235258/progress_20261027.json'):
        path=ROOT/name;old=json.loads(path.read_bytes());old.update(state);path.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block='''## CURRENT 2026-10-27: 수평 회전 시작 전 OFF 안정성 오중단 수정 v2

- `horizontal_quarter_trial_20261027_141354_998533`과 `...141539_818724`는 모두 모델 검사를 통과했지만 `OFF preload unstable; no enable`로 중단됐다. 샘플과 enable 기록은0개이고 설정 쓰기 및 원위치 hold 사전 설정 뒤 토크 OFF/설정 복구가 확인됐다. 두 사전 읽기 사이 ID10의 현재값과 목표값이 함께±2틱 바뀌었다. 카메라는 각각369/357장, 검은 화면0장이다. 실제 회전이나 큐브 파지는 시작하지 않았다.
- 공유 runner는 수정하지 않고 전용 수평 runner에서 OFF 사전 설정 확인만 바꿨다. 기존 시작 정지 읽기와 같은3회/범위2틱 조건을250ms 시간 예산 안에서 확인한다. 원래 OFF/목표4틱 창, 각 읽기의 OFF/오류/워치독/전압/온도/PID/전류/프로필 검사와 검토된 출발점4틱 한도는 유지한다. 목표나 설정 재전송은0회다. 토크 활성화부터 실제 이동과 정리까지 코드는 원본과 같다.
- 새14:20 PING/READ3회는 최대2틱 범위, 전부 OFF/오류0/워치독0, 최소4.5V, 최고23도였다. v2 현재 복귀210개를 다시 검사했다. loaded 경로와 기하201/물리27 검토는 해시가 동일하다. 저장된 두 실패 사례의 추가 읽기, 실제로 계속 움직이는 경우의 거부, 건강/설정 오류 및 시간 초과/취소, 활성화 전 실패의 OFF 정리를 포함한 총42개 무장치 검사가 통과했다.
- 다음 사용자 작업은 같은 `leap_horizontal_quarter_trial.cmd`를 한 번 실행해 `HORIZONTAL QUARTER-TURN v2` 헤더를 확인하는 것이다. 빈손 첫 SPACE, PLACE 뒤 배치/손 뺀 뒤 두 번째 SPACE, 자동 열기 후 큐브 제거와 세 번째 SPACE다. 완료/중단 뒤 했어만 받는다. 새 치수나 촬영, 손 모양 수동 복원을 요구하지 않는다.
- 근거: `right_hand_rotation_20261027/horizontal_quarter_trial_v2/readiness.json`, `review.json`, `preload_fix_review.json`, `operator_guide.md`. 실물 수평90도는 미검증이며 수직 성공 총2회는 유지한다. 이번 assistant 동작은 읽기와 오프라인 수정/검사뿐이고 모터 목표/토크/설정 쓰기는0회다.

<!-- END HORIZONTAL OFF PRELOAD FIX 20261027 -->

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8-sig')
        if '<!-- END HORIZONTAL OFF PRELOAD FIX 20261027 -->' in text:raise ValueError('Already recorded')
        path.write_text(block+text.replace('## CURRENT 2026-10-27:','## PREVIOUS 2026-10-27:',1),encoding='utf-8')
    for name,wanted in {'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b','right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716','exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0'}.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==wanted,name
    print(json.dumps(dict(status=state['status'],tests_passed=42,loaded_motion_unchanged=True,shared_runner_unchanged=True,next_user_command=state['next_user_command'],assistant_motor_writes=0),ensure_ascii=False))
if __name__=='__main__':main()
