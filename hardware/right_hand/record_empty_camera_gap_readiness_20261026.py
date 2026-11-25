"""Publish device-free verification of the scoped EMPTY image-gap recovery."""
import datetime as dt,hashlib,json
from pathlib import Path
from thumb_clear_root_empty_plan_20261026 import ROOT,FIXTURE,ThumbRootEmptyPlan,extra_hashes
from leap_thumb_clear_root_empty_20261026 import require_review
from empty_camera_gap_hold_20261026 import metadata

BASE=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454'
def read(p):return json.loads(p.read_bytes())
def write(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    # This checks the complete parent review chain as well as the new proof.
    # It opens no camera/serial port and sends no motor instructions.
    print('Verifying complete saved review chain; no devices opened.',flush=True)
    proof=require_review()
    plan=ThumbRootEmptyPlan(FIXTURE/'request.json');raw=plan.source_position();rows=plan.plan(raw)
    request=read(FIXTURE/'request.json');source=ROOT/request['readonly_source']
    if sha(source)!=request['readonly_source_sha256']:raise ValueError('Read-only source changed')
    process=read(FIXTURE/'off_process_review.json')
    if not process['wrapper_completed'] or process['hardware_commands_sent'] or process['devices_opened']:
        raise ValueError('OFF process proof is incomplete')
    if (len(proof['static_screens']),len(proof['dynamic_sensitivity']),len(proof['camera_gap_hold_sensitivity']))!=(201,9,9):
        raise ValueError('Incomplete native sensitivity cases')
    previous_failures=[]
    for name in ('thumb_clear_root_empty_20261026_105657_806418','thumb_clear_root_empty_20261026_110112_326785',
                 'thumb_clear_root_empty_20261026_110554_317085','thumb_clear_root_empty_20261026_111118_102113'):
        r=read(ROOT/name/'result.json')
        previous_failures.append(dict(folder=name,error=r.get('error'),failed_phase=r.get('failed_phase'),
            goal_write_events=len(r.get('write_events',[])),sample_count=len(r.get('samples',[])),
            all_torque_off_verified=r.get('all_torque_off_verified')))
    protected={ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
        ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
        Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'):'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'}
    poster=ROOT/'exhibition_20261023/족자_붙여넣기.txt'
    expected=read(ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json')['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    protected[poster]=expected['sha256'] if isinstance(expected,dict) else expected
    if any(sha(p)!=expected for p,expected in protected.items()):raise ValueError('Protected file changed')
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    ready=dict(timestamp=stamp,ready=True,command='leap_thumb_clear_root_empty.cmd',
        plan_version=plan.PLAN_VERSION,empty_hand_only=True,loaded_motion_included=False,
        physical_ninety_degree_rotation_verified=False,webcam_blacking_root_cause_identified=False,
        usb_ports_or_wiring_changed=False,camera_driver_settings_changed=False,
        camera_format_resolution_and_exposure_unchanged=True,camera_hold_policy=metadata(),
        hardware_health_limits_unchanged=True,actual_undervoltage_cutoff_unchanged=True,
        planned_goal_positions_and_order_unchanged_for_same_start=True,
        nominal_goal_equivalence_old_start_tested=True,goal_digest=plan.plan_digest(raw),
        planned_goal_count=len(rows),nominal_motion_seconds=len(rows)*plan.SAMPLE_PERIOD,
        motion_timeout_seconds=plan.ON_TIMEOUT,off_screen_timeout_seconds=240,
        support_escape_model_extra_seconds=plan.SAMPLE_PERIOD,support_depth_limits_unchanged=True,
        support_change_reason='One response case separated at6.46s versus6.4s; allow one75ms control cycle, no depth/health relaxation.',
        source_request=(FIXTURE/'request.json').relative_to(ROOT).as_posix(),readonly_source=request['readonly_source'],
        previous_failed_runs=previous_failures,actual_off_process_review=process,
        native_static_cases=201,native_response_cases=9,native_camera_hold_response_cases=9,
        stress_hold_scope='Two5s fixed-goal holds at observed LIFT91 and RETURN55 interruption locations; nine front/gain cases.',
        program_tests_passed=27,
        tests=dict(camera_recovery_health_cancel_deadlines_and_real_runner=13,motion_shutdown_resolution_and_proof=9,off_process_timeout_cancel_and_same_goals=5),
        test_source_hashes={name:sha(ROOT/name) for name in ('test_empty_camera_gap_hold_20261026.py','test_thumb_clear_root_empty_20261026.py','test_thumb_root_off_screen_process_20261026.py')},
        full_parent_review_chain_verified=True,native_proof_sha256=sha(FIXTURE/'screen.json'),code_hashes=extra_hashes(),
        protected_files_verified={str(p):value for p,value in protected.items()},
        assistant_motor_goal_writes=0,assistant_torque_changes=0,assistant_motor_setting_changes=0,
        next_action='Same leap_thumb_clear_root_empty.cmd once; keep cube OUT throughout and outside camera. Terminal SPACE once. Fresh OFF-pose simulation precedes about2-3min EMPTY motion; no PLACE. A brief image loss holds the last goal and continues live motor checks; three fresh usable frames resume the queue. A gap over5s,10s total hold budget, motor fault, Q or180s ON deadline stops. No camera reopen or goal resend while ON.')
    write(FIXTURE/'readiness_review.json',ready)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        path=ROOT/name;r=read(path);r.update(timestamp=stamp,thumb_root_proposal_readiness=ready,
            next_user_command=ready['command'],next_user_command_ready=True,next_user_command_empty_only=True,
            next_user_command_photo_only=False,next_user_command_has_loaded_motion=False,rotation_in_next_command=False,
            next_action=ready['next_action'],physical_ninety_degree_rotation_verified=False)
        write(path,r)
    block='''## CURRENT 2026-10-26: 빈손 동작의 순간 검정 영상은 목표 정지 및 실시간 모터 확인 후 복구

- 사용자 웹캠은 별도 USB 장치이고 노트북 두 포트는 로봇과 웹캠이 각각 사용한다. 포트와 케이블, 전원 배선 및 카메라 설정은 바꾸지 않았다. `105657`, `110112` 실행은 정상 영상 뒤 모든 픽셀이 0인 검정 프레임이 약 초당 1장으로 들어오면서 기존 0.5초 기준에 걸렸다. 40초 카메라 단독 확인은 정상 782장이고 검정 0장이었다. 간헐적인 검정 영상의 원인은 아직 확정하지 못했다.
- 새 전용 빈손 모드는 영상이 불안정하거나 오래되면 새 목표를 보내지 않고 마지막 목표를 유지한다. 대기 중에도 기존 전압과 전류, 추종, 온도, 고정 관절 및 통신 상태를 계속 읽고 검증한다. 서로 다른 정상 영상 3장이 모이고 최신 영상이 0.3초 이내면 다음 목표부터 이어간다. 한 번 최대 5초, 전체 대기 합계 최대 10초이며 대기 시간도 기존 180초 구동 제한에 포함한다. 지속적인 끊김이나 Q, 실제 모터 상태 이상은 OFF한다. 목표 재전송과 건너뛰기, 토크 ON 중 카메라 재시작은 없다. 이 변경은 `leap_thumb_clear_root_empty.cmd`에만 적용하며 큐브를 올리는 실행은 포함하지 않는다.
- 이후 `110554`, `111118` 중단은 카메라가 정상인 상태에서 동작 전 받침 접촉 이탈의 시뮬레이션 시한에 걸린 별개 문제였다. 약한 응답 조건 하나에서 6.4초 대신 6.46초에 이탈했으므로 모델 판정에 75ms 제어 주기 한 번을 더 허용했다. 접촉 깊이와 실제 전압 및 전류, 추종 한계는 유지했다. 현재 기준은 `finish_empty_readonly_20261026_112243_132406/result.json`이며 실행 때 새 OFF 자세를 다시 검사한다.
- 코드 검사 27개와 독립 검증을 포함한 전체 검증 연결이 통과했다. 현재 자세와 편차의 정적 조건 201개, 정상 응답 조건 9개 및 관측된 끊김 위치 두 곳에서 각각 5초 목표 대기를 넣은 응답 조건 9개가 통과했다. 전체 OFF 계산과 독립 검증은 약 134초로 기존 240초 제한 안에 완료됐다. 모터 목표의 위치와 순서는 같은 출발 자세에서 이전과 같음을 검사했다. 새 기준 자세에서는 목표 1,753개, 명목 동작 시간 약 131.5초다. 모의 대기 테스트와 모델 통과이며 실제 웹캠 복구 및 실물 90도 회전 성공을 뜻하지 않는다.
- 다음은 같은 **`leap_thumb_clear_root_empty.cmd`**를 한 번 실행한다. 큐브는 처음부터 끝까지 손과 카메라 밖에 두며 터미널 SPACE 한 번이다. PLACE 단계는 없다. 검사 후 약 2~3분 동안 빈손 동작을 하고 자동 OFF한다. 완료 후에는 ‘했어’라고 알려주면 저장 파일을 확인한다. 전시 문구 및 원본과 관절 범위 파일은 보존했고 이번 개발 및 검증에서 실제 모터 명령 쓰기와 설정 변경은 0회다.
- 근거: `thumb_root_empty_fixture_v7_camera_hold/readiness_review.json`, `off_process_review.json`, `screen.json`, `test_empty_camera_gap_hold_20261026.py`.

<!-- END EMPTY CAMERA GAP CURRENT 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END EMPTY CAMERA GAP CURRENT 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT ','## PREVIOUS ',1);path.write_text(block+old,encoding='utf-8')
    print(json.dumps(dict(ready=True,command=ready['command'],tests=27,static=201,normal_response=9,camera_hold_response=9,
        native_review_seconds=process['wall_seconds'],planned_goals=len(rows),motor_commands_sent=0),ensure_ascii=False),flush=True)
if __name__=='__main__':main()
