"""Record completed timing fix and the separate blocked current-pose review."""
import datetime as dt,json
from record_empty_camera_gap_readiness_20261026 import ROOT,FIXTURE,read,write,sha
from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan,extra_hashes
from thumb_clear_root_empty_gate_20261026 import validate_proof
from fresh_voltage_sample_20261026 import metadata
def main():
    plan=ThumbRootEmptyPlan(FIXTURE/'request.json')
    try:validate_proof(plan,FIXTURE/'screen.json')
    except RuntimeError as exc:rejection=str(exc)
    else:raise RuntimeError('Unexpected geometry approval; inspect evidence before publishing')
    stop=ROOT/'thumb_clear_root_empty_20261026_115050_905503/result.json';r=read(stop);e=r['voltage_events'][0]
    recovered=r['samples'][-2:]
    if not all(min(s['voltage_v'])>=3.8 for s in recovered):raise ValueError('No two recovered voltage readings')
    protected=read(ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454/thumb_root_empty_fixture_v7_camera_hold/readiness_review.json')['protected_files_verified']
    from pathlib import Path
    if any(sha(Path(path))!=expected for path,expected in protected.items()):raise ValueError('Protected source changed')
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    result=dict(timestamp=stamp,ready=False,command='leap_thumb_clear_root_empty.cmd',
        program_timing_fix_implemented=True,program_tests_passed=35,
        tests=dict(voltage_and_sampler=9,camera_hold_and_runner=13,motion_and_resolution=8,off_process_and_goal_equivalence=5),
        actual_stop=stop.relative_to(ROOT).as_posix(),actual_stop_sha256=sha(stop),
        actual_error=r['error'],minimum_logged_voltage_v=3.6,confirmation_elapsed_ms=e['elapsed_ms'],
        recovered_voltage_seen_twice=True,last_confirmation_was_past_deadline=True,
        voltage_thresholds_and150ms_budget_unchanged=True,first_healthy_followup_10ms_sleep_removed=True,
        fresh_voltage_read_policy=metadata(),code_hashes=extra_hashes(),
        actual_camera_good_frames=r['camera_recording']['image_signal_usable_frames'],actual_camera_blank_frames=0,
        actual_all_torque_off_verified=r['all_torque_off_verified'],actual_settings_restored=r['settings_restored'],
        failed_post_OFF_reference_reused=False,
        fresh_readonly_reference='finish_empty_readonly_20261026_120039_495713/result.json',
        current_geometry_model_accepted=False,current_native_rejection=rejection,
        current_model_issue=dict(pair='palm_collision_3|th_tip',nominal_mm=.009171367513747197,sensitivity_maximum_mm=.3080363113813587,
            physical_contact_confirmed_for_this_pose=False,physical_mesh_registration_verified=False,
            initial_contact_policy_scope_widened=False,collision_geometry_or_forces_disabled=False),
        pending_user_input='Whether the thumb tip black pad actually touches the palm in the current OFF posture; no new execution requested.',
        next_action='Wait for requested current thumb/palm contact observation; inspect a separate recovery/calibration route before authorizing another EMPTY trial. Do not repeat the command yet.',
        physical_ninety_degree_rotation_verified=False,assistant_motor_goal_writes=0,assistant_motor_setting_changes=0,
        protected_files_verified=protected)
    write(FIXTURE/'readiness_review.json',result)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        path=ROOT/name;old=read(path);old.update(timestamp=stamp,thumb_root_proposal_readiness=result,
            next_user_command=result['command'],next_user_command_ready=False,next_user_command_empty_only=True,
            next_user_command_has_loaded_motion=False,rotation_in_next_command=False,next_action=result['next_action'],
            physical_ninety_degree_rotation_verified=False)
        write(path,old)
    block='''## CURRENT 2026-10-26: 전압 회복 확인의 불필요한 대기 수정 / 현재 엄지 접촉 실물 확인 대기

- `thumb_clear_root_empty_20261026_115050_905503`는 빈손 받침 이탈 단계에서 최저 3.6V를 읽고 전압 회복 확인으로 중단됐다. 기록의 마지막 두 번은 전체 모터 3.8V 이상이었지만 마지막 확인 완료가 156ms여서 기존 150ms 제한을 넘었다. 영상 1,560장은 모두 정상이고 검정 프레임은 0장이다. 모터 토크 OFF 및 원래 설정 복원은 확인했다. 카메라 복구 대기는 이번 실행에서 발생하지 않았다.
- 이 빈손 모드만 첫 정상 전압을 읽은 직후의 10ms 지연을 없애 다음 측정을 바로 읽게 했다. 150ms 제한, 최대 6회 확인, 전체 3.8V 이상 연속 2회, 3.5V 이하 즉시 중단 및 반복 저전압 중단 기준은 유지한다. 기존 공유 샘플러는 보존했고 전용 버스가 새 샘플러를 선택한다. 목표 위치와 순서는 바꾸지 않았고 대기 중 목표를 재전송하지 않는다. 저장된 전압 순서와 31ms 읽기 시간을 재생하면 이전 샘플러의 시간 초과와 수정본의 시간 내 확인을 재현한다. 실제 전원 경로의 전압 하락이 해결됐다는 뜻은 아니다.
- 프로그램 검사 35개가 통과했다. 회복이 늦거나 한 번만 회복하는 경우, 계속 낮은 전압, 3.5V, 추종 이상, Q 및 반복 전압 하락은 여전히 중단한다. 카메라 복구와 실제 실행기의 중단 및 설정 복원 검사도 포함했다. 원본 시퀀스, 관절 범위 및 전시 문구는 보존했다. 모터를 구동하거나 설정을 쓰지 않았다.
- 이번 OFF 직후 기록은 약지 회전 관절이 9틱 이동해 안정된 기준으로 재사용하지 않았다. 대신 `finish_empty_readonly_20261026_120039_495713`에서 쓰기 없이 새 상태를 3번 읽었으며 최대 변화는 1틱이었다. 해당 자세의 별도 전체 모델 검사는 엄지 끝과 `palm_collision_3`의 초기 접촉이 기존 허용 범위의 표면 목록 밖이어서 미통과다. 모델상 겹침은 기준 자세 약 0.009mm, 각도 편차 포함 최대 약 0.308mm이며 실제 접촉이나 물리적 충돌을 확정한 값이 아니다. 초기 접촉 범위나 충돌 허용 깊이를 넓히지 않았다.
- 다음 동작 명령은 아직 재실행하지 않는다. 사용자에게 **현재 토크 OFF 자세에서 엄지 끝 검은 패드가 실제 손바닥에 닿아 있는지** 확인을 요청했다. 그 답으로 초기 접촉 분리 경로 또는 모델 정합 확인을 이어간다. 새 모델 결과가 미통과인 동안 실행 진입은 장치를 열기 전에 차단된다. 실물 90도 회전은 미검증이다.
- 근거: 최신 실행의 `saved_stop_review.json`, `thumb_root_empty_fixture_v8_voltage_fresh/readiness_review.json`, `screen.json`, `test_fresh_voltage_sample_20261026.py`.

<!-- END VOLTAGE FRESH PENDING CURRENT 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END VOLTAGE FRESH PENDING CURRENT 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT ','## PREVIOUS ',1);path.write_text(block+old,encoding='utf-8')
    print(json.dumps(dict(ready=False,timing_fix_complete=True,tests=35,awaiting='actual thumb tip/palm contact observation',motor_writes=0),ensure_ascii=False))
if __name__=='__main__':main()
