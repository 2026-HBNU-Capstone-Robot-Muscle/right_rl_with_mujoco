"""Publish the actual-failure regression and initial rigid-palm contact fix."""
import datetime as dt
from pathlib import Path
from record_empty_camera_gap_readiness_20261026 import ROOT,FIXTURE,read,write,sha
from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan,extra_hashes
from confirmed_thumb_rest_policy_20261026 import metadata
def main():
    chain=read(FIXTURE/'review_chain.json');proof=read(FIXTURE/'screen.json')
    if (chain.get('accepted') is not True or chain['code_hashes']!=extra_hashes()
        or chain['native_proof_sha256']!=sha(FIXTURE/'screen.json')
        or not chain['actual_failed_goal_digest_unchanged']):raise ValueError('Incomplete/changed full review chain')
    p=ThumbRootEmptyPlan(FIXTURE/'request.json');raw=p.source_position();rows=p.plan(raw)
    failed_folder=ROOT/'thumb_clear_root_empty_20261026_123524_339920';actual=read(failed_folder/'result.json')
    source=failed_folder/'fresh_off_request.json';request=read(source)
    if (actual.get('write_events') or actual.get('samples') or not actual['all_torque_off_verified']
        or request['off_state']['position']!=raw.tolist()):raise ValueError('Actual failed input does not match')
    if any(not m['initial_palm_body_identity']['verified_same_rigid_body'] for m in proof['model_manifests']):raise ValueError('Unverified palm geometry owners')
    previous=read(ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454/thumb_root_empty_fixture_v10_release_indices/readiness_review.json')
    protected=previous['protected_files_verified']
    if any(sha(Path(path))!=expected for path,expected in protected.items()):raise ValueError('Protected original changed')
    ready=previous.copy();stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    for key in ('added_initial_surface_pair','added_initial_surface_entry_limit_mm','readonly_source','strict_static_monotonic_release_and_no_new_deeper_contacts'):
        ready.pop(key,None)
    ready.update(timestamp=stamp,ready=True,plan_version=p.PLAN_VERSION,
        actual_failed_input=str(source.relative_to(ROOT)),actual_failed_input_sha256=sha(source),
        actual_failed_run=str(failed_folder.relative_to(ROOT)),actual_failed_hardware_writes=0,
        actual_failed_camera_usable_frames=958,actual_failed_camera_blank_frames=0,
        observed_start_drift_ticks=[-7,7,7,-6,0,0,0,0,0,0,0,1,0,0,0,0],
        reason_for_start_drift_identified=False,
        this_revision_goal_sequence_unchanged_from_actual_failed_trial=True,
        only_initial_palm_contact_identity_changed=True,contact_identity='same rigid palm body during initial rest/release only',
        original_overall4mm_initial_contact_limit_unchanged=True,
        maximum_body_contact_depth_must_monotonically_decrease=True,
        new_or_deeper_body_contacts_forbidden=True,other_finger_contacts_still_individually_checked=True,
        contact_transfer_between_collision_pieces_of_the_same_palm_allowed=True,
        raw_collision_geometry_or_contact_forces_changed=False,
        confirmed_thumb_rest_policy=metadata(raw),native_same_palm_body_evidence=[m['initial_palm_body_identity'] for m in proof['model_manifests']],
        initial_face_depth_records=[m['initial_raw_palm_face_depths_mm'] for m in proof['model_manifests']],
        initial_face_depth_record_scope='Before rigid-body grouping; pre-existing fine-contact classification unchanged.',
        source_request=str((FIXTURE/'request.json').relative_to(ROOT)),
        goal_digest=p.plan_digest(raw),planned_goal_count=len(rows),nominal_motion_seconds=len(rows)*p.SAMPLE_PERIOD,
        program_tests_passed=47,tests=dict(confirmed_escape_and_palm_body=11,motion_resolution_and_proof=9,
            camera_hold_and_runner=13,voltage_and_sampler=9,off_process_and_goal_equivalence=5),
        native_static_cases=chain['static_cases'],native_response_cases=chain['normal_cases'],native_camera_hold_response_cases=chain['hold_cases'],
        native_proof_sha256=chain['native_proof_sha256'],code_hashes=extra_hashes(),
        test_source_hashes={name:sha(ROOT/name) for name in ('test_confirmed_thumb_escape_20261026.py','test_thumb_clear_root_empty_20261026.py',
            'test_empty_camera_gap_hold_20261026.py','test_fresh_voltage_sample_20261026.py','test_thumb_root_off_screen_process_20261026.py')},
        actual_off_process_review=read(FIXTURE/'off_process_review.json'),
        assistant_motor_goal_writes=0,assistant_torque_changes=0,assistant_motor_setting_changes=0,
        next_action='Same leap_thumb_clear_root_empty.cmd once; leave fingers as they are. Cube OUT throughout and outside camera. One terminal SPACE; fresh OFF screen -> tiny thumb escape -> existing recovery/reference/new ending -> automatic OFF. No PLACE. Send done; saved result will be reviewed.')
    write(FIXTURE/'readiness_review.json',ready)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        path=ROOT/name;r=read(path);r.update(timestamp=stamp,thumb_root_proposal_readiness=ready,next_user_command=ready['command'],
            next_user_command_ready=True,next_user_command_empty_only=True,next_user_command_has_loaded_motion=False,
            rotation_in_next_command=False,next_action=ready['next_action'],physical_ninety_degree_rotation_verified=False)
        write(path,r)
    block='''## CURRENT 2026-10-26: 실제 실패 자세 재현 / 초기 손바닥 접촉을 같은 강체로 판정

- `thumb_clear_root_empty_20261026_123524_339920`는 모터 쓰기와 동작 샘플 모두 0회로 OFF 경로 검사에서 멈췄다. 토크 OFF는 확인했고 설정 전이므로 복원 플래그 false는 설정 복원 실패가 아니다. 영상 958장은 모두 정상이며 카메라나 전압 중단이 아니다. 앞선 기준보다 엄지 ID1/2/3/4가 -7/+7/+7/-6틱 달랐고 원인은 확정하지 않았다.
- 정적 검사 201개 중 27개에서 엄지 접촉이 손바닥의 4번 충돌 조각 쪽으로 옮겨갈 때 그 조각의 겹침 증가를 새 접촉 또는 더 깊은 접촉으로 처리했다. 정상 및 대기 응답 조건 18개는 이미 통과했다. 이번 수정은 처음부터 확인된 엄지와 손바닥 접촉만 강체 손바닥 기준으로 묶는다. 모든 손바닥 조각이 실제로 동일한 `palm` 몸체에 속하는지 각 모델에서 검증한다. 형상과 접촉 힘은 보존하며 기존 전체 초기 4mm 상한과 최대 손바닥 접촉 깊이의 단조 감소를 유지한다. 같은 손바닥 조각 사이의 접촉 이동은 허용하지만 다른 물체와 새 접촉, 더 깊은 전체 손바닥 접촉 및 분리 뒤 재접촉은 계속 거부한다. 초기 분리 이후에는 이 묶음을 적용하지 않는다.
- 방금 실패한 관절값과 목표 시퀀스 해시 그대로 재검증했다. 목표는 바꾸지 않았고 정적 201개, 정상 응답 9개와 고정 목표 대기 응답 9개가 통과했다. 전체 저장 검증 연결과 프로그램 검사 47개가 통과했다. 다른 강체의 조각을 묶으려는 경우, 다른 손가락 접촉, 깊어지는 접촉, 분리 뒤 재접촉 및 초기 분리 밖의 접촉을 포함한 거부 검사를 추가했다. 실제 회전 성공이나 정확한 실물/CAD 정합을 뜻하지 않는다.
- 다음은 같은 **`leap_thumb_clear_root_empty.cmd`**다. 손가락과 받침은 그대로 두고 큐브는 처음부터 끝까지 손과 카메라 밖에 둔다. 터미널 SPACE 한 번 후 새 OFF 자세 검사, 작은 엄지 분리, 기존 복귀 및 기준 동작과 새 종료 자세, 자동 OFF 순서다. PLACE 단계는 없다. 전압과 추종 및 전류 중단 기준, 150ms 전압 확인과 카메라 복구 대기는 유지한다. 완료 후 ‘했어’라고 알려주면 저장 파일을 확인한다.
- 이번 개발 중 실제 모터 구동 및 설정 쓰기는 0회다. 전시 문구, 원본과 관절 범위 파일은 보존했다. 근거: 실제 실패 실행의 `current_start_screen.json`, `thumb_root_empty_fixture_v11_palm_body/screen.json`, `review_chain.json`, `readiness_review.json` 및 `test_confirmed_thumb_escape_20261026.py`.

<!-- END PALM BODY CURRENT 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END PALM BODY CURRENT 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT ','## PREVIOUS ',1);path.write_text(block+old,encoding='utf-8')
    print(dict(ready=True,tests=47,static=201,normal=9,camera_hold=9,actual_failed_goals_unchanged=True,command=ready['command']))
if __name__=='__main__':main()
