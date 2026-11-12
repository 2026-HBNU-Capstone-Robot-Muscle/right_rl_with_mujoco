"""Record accepted initial-contact escape without changing parent certificates."""
import datetime as dt,json
from pathlib import Path
from record_empty_camera_gap_readiness_20261026 import ROOT,FIXTURE,read,write,sha
from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan,extra_hashes
from leap_thumb_clear_root_empty_20261026 import require_review
from confirmed_thumb_rest_policy_20261026 import metadata as contact_metadata,prefix
from fresh_voltage_sample_20261026 import metadata as voltage_metadata
from empty_camera_gap_hold_20261026 import metadata as camera_metadata
from camera_gap_native_policy_20261026 import expected_holds
def main():
    print('Checking full saved review chain; no devices opened.',flush=True)
    proof=require_review();p=ThumbRootEmptyPlan(FIXTURE/'request.json');raw=p.source_position();rows=p.plan(raw)
    process=read(FIXTURE/'off_process_review.json');request=read(FIXTURE/'request.json')
    if not process['wrapper_completed'] or process['hardware_commands_sent'] or process['devices_opened']:raise ValueError('Incomplete process proof')
    if sha(ROOT/request['readonly_source'])!=request['readonly_source_sha256']:raise ValueError('Read-only source changed')
    previous=read(ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454/thumb_root_empty_fixture_v7_camera_hold/readiness_review.json')
    protected=previous['protected_files_verified']
    if any(sha(Path(path))!=expected for path,expected in protected.items()):raise ValueError('Protected original changed')
    ready=previous.copy();stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    ready.update(timestamp=stamp,ready=True,plan_version=p.PLAN_VERSION,
        planned_goal_positions_and_order_unchanged_for_same_start=False,
        only_initial_release_route_modified=True,original_release_endpoint_and_subsequent_reference_preserved=True,
        initial_escape_goal_count=len(prefix(raw)),maximum_initial_thumb_travel_ticks=12,
        initial_thumb_contact_user_confirmed=True,exact_contact_mesh_registration_verified=False,
        confirmed_thumb_rest_policy=contact_metadata(raw),initial_contact_surface_scope_extended=True,
        added_initial_surface_pair='palm_collision_3|th_tip',added_initial_surface_entry_limit_mm=.35,
        original_overall4mm_initial_contact_limit_unchanged=True,
        strict_static_monotonic_release_and_no_new_deeper_contacts=True,
        collision_shapes_and_contact_forces_preserved=True,
        fresh_voltage_read_policy=voltage_metadata(),camera_hold_policy=camera_metadata(),
        stress_hold_scope=expected_holds(rows),
        actual_undervoltage_cutoff_and150ms_budget_unchanged=True,
        goal_digest=p.plan_digest(raw),planned_goal_count=len(rows),nominal_motion_seconds=len(rows)*p.SAMPLE_PERIOD,
        source_request=(FIXTURE/'request.json').relative_to(ROOT).as_posix(),readonly_source=request['readonly_source'],
        previous_actual_voltage_stop='thumb_clear_root_empty_20261026_115050_905503/result.json',
        program_tests_passed=40,
        tests=dict(voltage_and_sampler=9,camera_hold_and_runner=13,motion_shutdown_resolution_and_proof=9,
            off_process_and_goal_equivalence=5,confirmed_initial_escape=4),
        test_source_hashes={name:sha(ROOT/name) for name in ('test_fresh_voltage_sample_20261026.py','test_empty_camera_gap_hold_20261026.py',
            'test_thumb_clear_root_empty_20261026.py','test_thumb_root_off_screen_process_20261026.py','test_confirmed_thumb_escape_20261026.py')},
        native_static_cases=len(proof['static_screens']),native_response_cases=len(proof['dynamic_sensitivity']),
        native_camera_hold_response_cases=len(proof['camera_gap_hold_sensitivity']),
        actual_off_process_review=process,native_proof_sha256=sha(FIXTURE/'screen.json'),code_hashes=extra_hashes(),
        live_full_plan_rebuilding_for_release_indices_removed=True,
        assistant_motor_goal_writes=0,assistant_torque_changes=0,assistant_motor_setting_changes=0,
        next_action='Same leap_thumb_clear_root_empty.cmd once. Do not reshape fingers. EMPTY throughout; cube outside camera too. Terminal SPACE once; fresh OFF screen, tiny thumb escape, existing recovery/reference and new thumb/root ending, then automatic OFF. No PLACE or loaded motion. Report done; saved files will be reviewed.')
    write(FIXTURE/'readiness_review.json',ready)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        path=ROOT/name;r=read(path);r.update(timestamp=stamp,thumb_root_proposal_readiness=ready,
            next_user_command=ready['command'],next_user_command_ready=True,next_user_command_empty_only=True,
            next_user_command_has_loaded_motion=False,rotation_in_next_command=False,next_action=ready['next_action'],
            physical_ninety_degree_rotation_verified=False)
        write(path,r)
    block='''## CURRENT 2026-10-26: 현재 엄지의 손바닥 접촉 확인 / 약 1도 이내의 분리 경로 검증

- 사용자가 현재 엄지 끝 검은 패드가 실제 손바닥에 닿아 있다고 확인했다. `finish_empty_readonly_20261026_120039_495713`의 안정된 OFF 관절값과 이 답을 기준으로 자동 분리 경로를 계산했다. 접촉하는 정확한 CAD 면까지 실물 정합됐다는 뜻은 아니다. 손이나 받침을 다시 맞출 필요는 없다.
- 기존 손바닥 표면 6번과 7번 옆의 3번 표면에 이미 존재하는 초기 엄지 접촉만 이 빈손 모드에서 추가로 기록한다. 3번 표면의 진입 상한은 0.35mm이고 원래 전체 4mm 초기 접촉 상한은 유지한다. 확인한 자세 근처에서만 적용되며 최초 관절값과 각도 편차별 접촉 깊이 이상으로 더 누르거나, 동작 중 새 접촉이 생기거나, 분리된 접촉이 다시 생기는 경로는 통과시키지 않는다. 충돌 형상과 접촉 힘을 끄거나 지우지 않았다.
- 단일 관절의 직선 열기 75개 후보는 모든 각도 편차에서 통과하지 못했다. 대신 네 엄지 관절을 함께 조금씩 움직이는 6개 목표를 찾았다. 기준 대비 최댓값은 12틱, 약 1.05도다. 67개 시작 편차에서 각 접촉 깊이가 증가하지 않고 모두 분리된다. 이 목표 뒤 원래 초기 분리의 종료 목표에 합류하며, 이후 기존 복귀 및 전체 기준 동작과 새 엄지 후퇴 및 검지·중지 올리기 목표는 유지한다. 실제 실행은 아직 하지 않았다.
- 전체 정적 조건 201개와 정상 응답 9개, 영상 끊김을 흉내 낸 고정 목표 대기 응답 9개가 통과했다. 초기 분리 동안 관절별 확인 대상을 계산할 때 전체 경로를 반복 생성하던 작업도 제거했으며 목표 해시 동일성을 확인했다. 프로그램 검사 40개 및 독립적인 전체 검증 연결을 통과했다. 영상 복구의 5초/합계 10초 대기, 전압 확인의 150ms 및 정상값 2회, 실제 전압과 전류 및 추종 중단 기준은 유지했다.
- 다음은 **`leap_thumb_clear_root_empty.cmd`** 한 번이다. 손가락은 현재대로 두고 큐브는 처음부터 끝까지 손과 카메라 밖에 둔다. 터미널 SPACE 한 번이면 새 OFF 자세 검증 뒤 작은 엄지 분리와 빈손 경로를 실행하고 자동 OFF한다. PLACE 단계는 없고 약 2~3분의 빈손 동작이다. 완료 후 ‘했어’라고 알려주면 저장 파일을 확인한다. 실물 90도 회전은 미검증이다.
- 이번 개발 과정의 실제 모터 구동 및 설정 쓰기는 0회다. 전시 문구와 원본 및 관절 범위 파일은 보존했다. 근거: `confirmed_thumb_palm_contact_20261026.json`, `confirmed_thumb_escape_prefix_20261026.json`, `thumb_root_empty_fixture_v10_release_indices/readiness_review.json` 및 `screen.json`.

<!-- END CONFIRMED THUMB ESCAPE CURRENT 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END CONFIRMED THUMB ESCAPE CURRENT 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT ','## PREVIOUS ',1);path.write_text(block+old,encoding='utf-8')
    print(json.dumps(dict(ready=True,command=ready['command'],tests=40,static=ready['native_static_cases'],
        normal=ready['native_response_cases'],camera_hold=ready['native_camera_hold_response_cases'],
        prefix_goals=6,goals=len(rows),motor_writes=0),ensure_ascii=False),flush=True)
if __name__=='__main__':main()
