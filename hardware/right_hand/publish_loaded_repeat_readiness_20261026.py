"""Keep the first physical success; publish a timing-only repeat correction."""
import datetime as dt,hashlib,json,time
from pathlib import Path
import numpy as np
from loaded_repeat_plan_20261026 import ROOT,FIXTURE,LoadedRepeatPlan,extra_hashes,TURN_PHASE
from loaded_thumb_rotation_plan_20261026 import LoadedThumbRotationPlan
from loaded_repeat_voltage_20261026 import metadata as voltage_metadata
from repeat_support_model_resolution_20261026 import metadata as support_metadata
from leap_loaded_repeat_20261026 import require_review
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    print('Checking complete saved parent/repeat review chain; no devices opened.',flush=True)
    proof=require_review();process=read(FIXTURE/'off_process_review.json')
    assert process['accepted'] and process['hardware_commands_sent']==0 and not process['devices_opened']
    source=ROOT/'loaded_thumb_rotation_20261026_163144_150855/result.json';failed=read(source)
    assert failed['status']=='stopped' and failed['all_torque_off_verified'] and failed['settings_restored']
    previous=read(ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/loaded_thumb_root_fixture_v4_portable_proof/readiness_review.json')
    assert all(sha(Path(path))==expected for path,expected in previous['protected_files_verified'].items())
    p=LoadedRepeatPlan(FIXTURE/'request.json');raw=p.source_position();rows=p.plan(raw);old=LoadedThumbRotationPlan.plan(p,raw)
    assert [s for s,q in rows]==[s for s,q in old]
    np.testing.assert_array_equal([q for s,q in rows],[q for s,q in old])
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    command='leap_loaded_thumb_rotation_retry.cmd'
    action='Run leap_loaded_thumb_rotation_retry.cmd once. FIRST SPACE EMPTY. At PLACE use GREEN up/ORANGE toward WRIST ID41, hands clear, SECOND SPACE. Remove cube BEFORE THIRD SPACE. Same about96s loaded path. Send done; saved files will be reviewed.'
    ready=dict(timestamp=stamp,ready=True,command=command,plan_version=p.PLAN_VERSION,
        failed_repeat=source.parent.name,failed_repeat_sha256=sha(source),
        latest_repeat_completed=False,physical_flip_successes=1,loaded_attempts_observed=2,
        separate_COM_missing_preflight='loaded_thumb_rotation_20261026_162938_932327',
        physical_quarter_turn_demonstrated_once=True,repeatability_verified=False,
        failed_voltage_confirmation_elapsed_ms=failed['voltage_events'][-1]['elapsed_ms'],
        failed_confirmation_reached_healthy_twice_but_after_deadline=True,
        voltage_policy=voltage_metadata(),support_reporting_resolution=support_metadata(),
        host_clock_info={name:vars(time.get_clock_info(name)) for name in ('monotonic','perf_counter')},
        unchanged_goal_positions_order_and_75ms_pacing=True,hardware_current_PID_voltage_tracking_limits_unchanged=True,
        software_counterfactual_replay_only=True,physical_voltage_improvement_not_verified=True,
        newly_captured_off_source=read(FIXTURE/'request.json')['readonly_source'],
        planned_goals=len(rows),nominal_motion_s=len(rows)*p.SAMPLE_PERIOD,
        nominal_loaded_s=(sum(1 for s,q in rows[next(i for i,(s,q) in enumerate(rows) if s==TURN_PHASE):])+20)*p.SAMPLE_PERIOD,
        native_static_cases=len(proof['static_screens']),native_normal_cases=len(proof['dynamic_sensitivity']),
        native_empty_camera_hold_cases=len(proof['camera_gap_hold_sensitivity']),
        native_proof_sha256=sha(FIXTURE/'screen.json'),off_process_review=process,
        tests_passed=25,test_file_sha256=sha(ROOT/'test_loaded_repeat_20261026.py'),
        code_hashes=extra_hashes(),protected_files_verified=previous['protected_files_verified'],
        previous_successful_mode_files_unchanged=True,previous_recorded_success_preserved=True,
        assistant_motor_goal_writes=0,assistant_torque_changes=0,assistant_setting_writes=0,
        current_pose_read_only=True,cube_removed_operator_confirmed=True,next_action=action)
    write(FIXTURE/'readiness_review.json',ready)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json',
                 'right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json',
                 'right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        path=ROOT/name;r=read(path)
        r.update(timestamp=stamp,loaded_repeat_readiness=ready,latest_loaded_repeat_completed=False,
            latest_loaded_repeat_error=failed['error'],physical_forward_flip_successful_trials=1,repeatability_verified=False,
            next_user_command=command,next_user_command_ready=True,next_user_command_empty_only=False,
            next_user_command_has_loaded_motion=True,rotation_in_next_command=True,next_action=action)
        write(path,r)
    block='''## CURRENT 2026-10-26: 재현 실행 전압 확인 중단 및 읽기 시간 계산 수정

- `loaded_thumb_rotation_20261026_162938_932327`는 COM7 미인식으로 장치 설정/목표 쓰기 전 멈췄다. 이후 `163144_150855`는 큐브를 놓고 기준 회전 경로를 실행하다가 마지막 추가 동작 전에 전압 확인이 중단됐다. 당시 3.6V 관측 후 다섯 번의 읽기에서 3.7→3.8→3.7→3.8→3.8V가 관측됐지만, 연속 정상값 두 번이 모인 경과 시간이 172ms라 기존 150ms 제한을 넘었다. 영상 2,429장은 모두 정상이고 최대 전류 121mA, 최대 추종 오차 약 0.101rad다. 토크 OFF와 설정 복원을 확인했다. 사용자가 큐브를 뺐다고 확인했으며, 그 뒤 별도 읽기에서 현재 OFF 자세가 3회 최대 1틱 편차로 안정된 것을 확인했다.
- 이전 `135418_442474`의 실물 수직 뒤집기 성공 1회와 영상은 그대로 보존했다. 이번은 재현 완주가 아니므로 반복성은 미검증이다. 초기 COM 미연결을 제외한 큐브 실행 관측은 성공 1회와 중단 1회다.
- 현재 Windows Python의 `time.monotonic`은 GetTickCount64 기반 15.625ms 해상도이고, `time.perf_counter`는 QueryPerformanceCounter 기반 0.1µs 해상도다. 실제 읽기 전용 관측에서 전체 상태 읽기는 약 26.35~26.77ms였다. 새 전용 모드는 150ms 전압 확인만 정밀 시계로 재며, 정상값이 아닌 경우의 최소 10ms 읽기 간격에 실제 읽기 작업 시간을 포함한다. 따라서 이미 읽기로 10ms 이상을 쓴 경우 추가로 10ms를 기다리지 않는다. 이벤트 시각은 기존 모터 구동 시각 기준을 유지하여 서로 다른 시계의 원점을 빼지 않는다. 목표 전송/재전송은 없고 두 번의 전체 ID 3.8V 이상 확인, 3.5V 이하 즉시 중단, 최대 6회 읽기와 2초 안 3회 저전압 중단을 모두 유지한다.
- 저장된 실패 전압 순서와 기록된 읽기 소요 시간을 이용한 재생 검사에서 이전 방식은 제한을 넘고 새 방식은 약 141ms 안에 두 번의 정상값을 얻었다. 이는 소프트웨어 반사실 재생이며, 읽는 시각이 달라졌을 때 실제 전압도 같은 순서로 회복한다는 보장은 아니다. 전원 배선이나 실제 전압 하한, 전류 및 PID와 추종 기준은 바꾸지 않았다. 이전 성공 코드도 그대로 두고 새 실행 파일을 분리했다.
- 현재 OFF 자세 검사에서 약한 모델 응답 한 조건의 받침 모서리 잔여 접촉 약 0.000570mm가 이탈 시한에 남아 있었다. 알려진 받침 충돌 쌍에만 0.001mm 보고 해상도를 명시해 이보다 작은 모델 값을 0으로 보고하고 원래 형상/접촉 힘과 이탈 시한은 보존했다. 더 큰 값과 알려지지 않은 충돌 쌍은 계속 검사하며, 버린 값의 최대치를 기록하고 독립 검사한다. 정적 조건 201개, 정상 응답 9개 및 빈손 영상 대기 응답 9개가 통과했다. 프로그램 검사 25개는 실제 실패값 재생, 정밀 시계 원점 분리, 늦은 회복과 지속 저전압 및 다른 모터 이상/Q 중단, 샘플러 연결과 동일 목표 경로, PLACE/REMOVE 및 종료 복원, 받침 보고 해상도 한계와 변경된 전압 검증 결과의 거부를 포함한다.
- 다음은 **`leap_loaded_thumb_rotation_retry.cmd`** 한 번이다. 첫 SPACE는 빈손, PLACE 안내 후 초록 위/주황 손목 ID41 방향으로 큐브를 놓고 손을 치운 뒤 두 번째 SPACE, REMOVE 안내 후 큐브를 뺀 다음 세 번째 SPACE다. 큐브 동작은 이전과 같은 약 96초이며 준비 경로와 속도 역시 같은 출발 자세에서 동일하다. 완료 후 ‘했어’라고 알려주면 저장 파일을 읽는다.
- 근거: 중단 실행 `result.json`, `repeat_off_readonly_20261026_164534_888040/result.json`, `loaded_repeat_fixture_v2_precise_support_resolution/screen.json`, `off_process_review.json`, `readiness_review.json`, `test_loaded_repeat_20261026.py`. 이번 개발은 모터 목표/토크/설정 쓰기 0회이며 현재 상태만 읽었다. 전시 문구와 원본 시퀀스 및 관절 범위 파일은 보존했다.

<!-- END PRECISE REPEAT CURRENT 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END PRECISE REPEAT CURRENT 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT ','## PREVIOUS ',1);path.write_text(block+old,encoding='utf-8')
    assert sha(source)==ready['failed_repeat_sha256']
    assert all(sha(Path(path))==expected for path,expected in ready['protected_files_verified'].items())
    print(json.dumps(dict(ready=True,command=command,tests=25,static=201,dynamic=9,camera_hold=9,same_goals=True,physical_repeat_verified=False)),flush=True)
if __name__=='__main__':main()
