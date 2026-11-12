"""Record the OFF-only timeout fix, equivalent goals and completed checks."""
import datetime as dt,hashlib,json
from thumb_clear_root_empty_plan_20261026 import ROOT,FIXTURE,ThumbRootEmptyPlan,extra_hashes
from thumb_clear_root_empty_gate_20261026 import validate_proof
from thumb_root_off_screen_process_20261026 import MAX_WALL_SECONDS

BASE=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454'
PREVIOUS=BASE/'thumb_root_empty_fixture_v5_corner_resolution'
def read(p):return json.loads(p.read_bytes())
def write(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    p=ThumbRootEmptyPlan(FIXTURE/'request.json');raw=p.source_position();rows=p.plan(raw)
    proof=validate_proof(p,FIXTURE/'screen.json');basis=read(PREVIOUS/'before_timeout_fix/basis.json')
    if p.plan_digest(raw)!=basis['plan_digest'] or [[phase,q.tolist()] for phase,q in rows]!=basis['goal_sequence']:
        raise ValueError('Motor goals changed')
    actual=ROOT/'thumb_clear_root_empty_20261026_102320_074990'
    result=read(actual/'result.json');request=read(actual/'fresh_off_request.json')
    if (result.get('error')!='OFF path screen exceeded 120s; no writes' or result.get('write_events')
        or result.get('samples') or not result.get('all_torque_off_verified') or request['off_state']['position']!=raw.tolist()):
        raise ValueError('Actual stop/source does not match reviewed timeout')
    process=read(FIXTURE/'off_process_review.json')
    if not process['wrapper_completed'] or process['hardware_commands_sent'] or process['devices_opened']:
        raise ValueError('Actual process wrapper not verified')
    protected={ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
               ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716'}
    poster=ROOT/'exhibition_20261023/족자_붙여넣기.txt';expected=read(ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json')['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    if isinstance(expected,dict):expected=expected['sha256']
    protected[poster]=expected
    if any(sha(path)!=expected for path,expected in protected.items()):raise ValueError('Protected file changed')
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    ready=read(PREVIOUS/'readiness_review.json');ready.update(timestamp=stamp,ready=True,
        command='leap_thumb_clear_root_empty.cmd',off_screen_timeout_seconds=MAX_WALL_SECONDS,
        previous_off_screen_timeout_seconds=120,off_only_timeout_change=True,
        actual_failed_run=actual.name,actual_failure_had_no_motor_writes=True,
        all_goal_positions_and_order_unchanged=True,goal_digest=p.plan_digest(raw),
        goal_construction_repeated_tail_work_removed=True,program_tests_passed=14,
        tests=dict(motion_and_shutdown=6,corner_resolution=2,independent_proof=1,off_process_timeout_cancel_and_same_goals=5),
        program_test_source_sha256=sha(ROOT/'test_thumb_clear_root_empty_20261026.py'),
        off_process_test_source_sha256=sha(ROOT/'test_thumb_root_off_screen_process_20261026.py'),
        actual_off_process_review=process,code_hashes=extra_hashes(),native_proof_sha256=sha(FIXTURE/'screen.json'),
        native_static_cases=len(proof['static_screens']),native_response_cases=len(proof['dynamic_sensitivity']),
        source_request=FIXTURE.relative_to(ROOT).as_posix()+'/request.json',
        planned_goal_count=len(rows),nominal_motion_seconds=len(rows)*p.SAMPLE_PERIOD,
        motion_timeout_seconds=p.ON_TIMEOUT,hardware_health_limits_unchanged=True,
        assistant_motor_goal_writes=0,assistant_torque_changes=0,
        next_action='Same leap_thumb_clear_root_empty.cmd, once. EMPTY throughout, cube outside camera too. Startup review prints before camera. Terminal SPACE once starts fresh OFF/current-pose screen, now up to240s with cancellation. Only a completed accepted proof permits unchanged2029-goal route, about2min35s of motion; automatic OFF. No PLACE.')
    write(FIXTURE/'readiness_review.json',ready)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        path=ROOT/name;r=read(path);r.update(timestamp=stamp,thumb_root_proposal_readiness=ready,
            next_user_command=ready['command'],next_user_command_ready=True,next_user_command_empty_only=True,
            next_user_command_photo_only=False,next_user_command_has_loaded_motion=False,rotation_in_next_command=False,
            next_action=ready['next_action'],physical_ninety_degree_rotation_verified=False)
        write(path,r)
    block='''## CURRENT 2026-10-26: 빈손 검사 120초 제한으로 중단 / OFF 검사만 240초 및 계산 중복 제거

- `thumb_clear_root_empty_20261026_102320_074990`는 모터 동작 전에 `OFF path screen exceeded 120s; no writes`로 중단됐다. -15mm와 -10mm의 응답 조건 6개는 로그에서 통과했으며 나머지를 계산하던 중 프로세스가 시간 제한에 걸렸다. 실제 명령 쓰기와 동작 샘플은 모두 0회이고 16개 토크 OFF를 확인했다. 설정 전이라 복원 플래그 false는 복원 실패를 의미하지 않는다. 카메라 959장은 모두 정상이었다.
- 전용 OFF 계산 프로세스의 제한을 120초에서 240초로 늘렸다. 기존 공유 대기 함수는 보존했고 이 빈손 모드만 새 함수를 쓴다. Q와 오류 및 제한 시간 초과 때 자식 프로세스를 종료하며, 불완전하거나 기존 결과를 재사용해서 움직이지 않는다. 시작 저장 검증 중임을 카메라가 열리기 전에도 출력한다. 실제 구동 제한 180초와 전압·전류·추종 및 카메라 기준은 바꾸지 않았다.
- 계획의 종료 자세 계산을 정지 목표 60개마다 반복하던 중복을 제거했다. 2,029개 목표 위치와 단계 순서는 이전과 동일하며 전체 목표 해시는 `08ad21df6abd76bc3775b9c59bbf85f0f0d01166c311cd86d54d175563f24ba8`이다. 실패 실행의 현재 관절값도 검증 입력과 정확히 같다. 새 실제 OFF 프로세스의 전체 계산과 독립 검증이 완료됐고 정적 201개와 응답 9개가 통과했다. 구간 대기·취소·시간 초과·기존 결과 거부 및 목표 동일성을 포함한 프로그램 검사 총 14개가 통과했다.
- 다음 명령은 같은 **`leap_thumb_clear_root_empty.cmd`**다. 큐브는 처음부터 끝까지 손과 카메라 밖이며 터미널 SPACE 한 번이다. PLACE 단계는 없다. OFF 경로 검사 후 약 2분35초 움직이고 자동 OFF한다. 이번 수정은 검사 시간 문제를 고친 것이며 실물 회전 성공은 아직 미검증이다. 실제 모터 쓰기는 0회이고 전시 문구 및 원본과 범위 파일은 보존했다.
- 근거는 `slow_run_0454/review_thumb_clear_root_empty_20261026_102320_074990/review.json`과 `thumb_root_empty_fixture_v6_off_timeout/off_process_review.json`, `readiness_review.json`이다.

<!-- END THUMB ROOT OFF TIMEOUT CURRENT 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END THUMB ROOT OFF TIMEOUT CURRENT 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT ','## PREVIOUS ',1);path.write_text(block+old,encoding='utf-8')
    print(json.dumps(dict(ready=True,command=ready['command'],old_off_timeout=120,new_off_timeout=MAX_WALL_SECONDS,
        motor_goals_unchanged=True,static_cases=201,response_cases=9,tests=14,motor_writes=0),ensure_ascii=False))
if __name__=='__main__':main()
