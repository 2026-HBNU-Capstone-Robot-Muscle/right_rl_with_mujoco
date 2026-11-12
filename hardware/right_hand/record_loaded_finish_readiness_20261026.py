"""Preserve EMPTY evidence and record the next supervised cube experiment."""
import hashlib,json
from datetime import datetime,timezone,timedelta
from pathlib import Path
import numpy as np
from loaded_quarter_finish_plan_20261025 import LoadedQuarterFinishPlan,ROOT,FIXTURE,loaded_basis,ACTUAL,REPLAY,TURN_PHASE,FINISH_PHASE
from loaded_quarter_finish_gate_20261025 import validate_proof
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_bytes())
def write(p,value):Path(p).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    from leap_loaded_quarter_finish_trial_20261025 import require_offline_review
    proof=require_offline_review();plan=LoadedQuarterFinishPlan(FIXTURE/'request.json');basis=loaded_basis();actual=read(ACTUAL);samples=actual['samples'];stamp=datetime.now(timezone(timedelta(hours=9))).isoformat()
    summary=dict(timestamp=stamp,source=ACTUAL.relative_to(ROOT).as_posix(),source_sha256=sha(ACTUAL),status=actual['status'],all_torque_off_verified=actual['all_torque_off_verified'],settings_restored=actual['settings_restored'],
        accepted_reference_goals=sum(s['phase']=='stage_6_empty_quarter_reference' for s in samples),accepted_new_finish_goals=sum(s['phase']=='stage_7_empty_root_finish' for s in samples),sample_count=len(samples),motor_write_events=len(actual['write_events']),
        max_tracking_rad=float(max(max(s['tracking_rad']) for s in samples)),peak_current_ma=int(max(np.max(abs(np.array(s['current_ma']))) for s in samples)),min_voltage_v=min(min(s['voltage_v']) for s in samples),max_temperature_c=max(max(s['temperature_c']) for s in samples),
        camera_images=actual['camera_recording']['frame_count'],camera_usable_images=actual['camera_recording']['image_signal_usable_frames'],camera_blank_images=actual['camera_recording']['image_signal_blank_frames'],
        final_goal_error_ticks=(np.array(samples[-1]['actual'])-np.array(samples[-1]['goal'])).tolist(),
        final_off_reference_valid=actual['final_off_reference_valid'],post_off_immediate_readings_not_used_as_stable_start=True,stable_followup_readonly='finish_empty_readonly_20261025_235113_620117/result.json',
        first_space_wait_resumed=actual['camera_first_space_waits'][0]['resumed'],first_space_wait_seconds=actual['camera_first_space_waits'][0]['elapsed_seconds'],
        physical_cube_rotation_tested=False,physical_collision_absence_verified=False,visible_human_finger_at_base_in_selected_frames=True,visible_cube_in_selected_frames=False)
    out=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/finish_empty_completion_2345';write(out/'summary.json',summary)
    prior_path=out/'completion_review.json';prior=read(prior_path)
    for item in prior:
        source=ROOT/item['source']/'result.json';r=read(source);item['write_count']=len(r.get('write_events',[]))
    write(prior_path,prior)
    replay=read(REPLAY);rows=plan.plan(plan.source_position())
    readiness=dict(timestamp=stamp,user_command='leap_loaded_quarter_finish_trial.cmd',supervised_experimental_loaded_trial_ready=True,robust_loaded_controller_verified=False,physical_cube_rotation_verified=False,
        actual_empty_completion=summary,conditional_measured_empty_replay=basis,conditional_cube_overlap_max_mm=max(s['max_cube_overlap_mm'] for s in replay['cases']),
        conditional_cube_red_up_error_max_deg=max(s['red_up_error_deg'] for s in replay['cases']),conditional_cube_green_wrist_error_max_deg=max(s['green_wrist_error_deg'] for s in replay['cases']),
        native_fixture=FIXTURE.relative_to(ROOT).as_posix(),native_proof_sha256=sha(FIXTURE/'screen.json'),plan_sha256=plan.plan_digest(plan.source_position()),code_hashes=proof['extra_code_hashes'],
        static_hand_route_cases=len(proof['static_screens']),native_dynamic_hand_route_cases=len(proof['dynamic_sensitivity']),native_screen_includes_cube=False,
        program_tests_passed=9,program_test_source_sha256=sha(ROOT/'test_loaded_quarter_finish_20261026.py'),source_stable_off='finish_empty_readonly_20261025_235113_620117/result.json',
        total_goal_count=len(rows),nominal_goal_seconds=len(rows)*.05,loaded_reference_goals=sum(p==TURN_PHASE for p,q in rows),loaded_finish_goals=sum(p==FINISH_PHASE for p,q in rows),loaded_motion_seconds=41.2,
        initial_thumb_mcp_measured_following_required=True,initial_thumb_mcp_following_limit_ticks=20,
        PID_current_rate_range_and_actual_health_stops_unchanged=True,first_space_camera_wait_fix_reused=True,
        placement_checkpoint_required=True,removal_checkpoint_required=True,terminal_space_count=3,automatic_motion_retry=False,live_SAC_policy=False,
        physical_gravity_camera_registration_and_contact_parameters_not_identified=True,
        assistant_read_only_acquisition=True,assistant_motor_goal_writes=0,assistant_torque_changes=0,
        next_action='EMPTY at first terminal SPACE. Wait for preparation and PLACE. GREEN up, ORANGE toward wrist/ID41 on flat index/middle support; hands clear, second SPACE starts reference+finish after countdown. REMOVE Cube before third SPACE/OFF. Send done for saved-file review.')
    protected={ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'):'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'}
    if any(sha(p)!=v for p,v in protected.items()):raise ValueError('Protected original changed')
    poster=ROOT/'exhibition_20261023/족자_붙여넣기.txt';baseline=read(ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json')['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    if isinstance(baseline,dict):baseline=baseline['sha256']
    if sha(poster)!=baseline:raise ValueError('Poster changed')
    readiness['poster_and_originals_unchanged']=True;write(FIXTURE/'readiness_review.json',readiness)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        p=ROOT/name;r=read(p);r.update(timestamp=stamp,empty_reference_plus_finish_physically_completed=True,latest_empty_finish_completion=summary,loaded_finish_readiness=readiness,
            next_user_command=readiness['user_command'],next_user_command_ready=True,next_user_command_empty_only=False,next_user_command_photo_only=False,next_user_command_has_loaded_motion=True,rotation_in_next_command=True,
            next_loaded_reference_goal_count=824,next_loaded_motion_seconds=41.2,physical_ninety_degree_rotation_verified=False,next_action=readiness['next_action'])
        if name.endswith('loaded_quarter_trial_20261025/readiness_review.json'):
            r.update(supervised_experimental_loaded_trial_ready=False,original_38s_only_command_superseded=True,new_loaded_finish_trial_ready=True)
        else:r['supervised_experimental_loaded_trial_ready']=True
        write(p,r)
    text='''## CURRENT 2026-10-26: 새 마무리 포함 빈손 완료 / 큐브를 올리는 후속 실험 준비

- 23:45 첫 시도는 ID1의 초기 응답 시간 초과로 쓰기 없이 종료됐다. 이어진 `quarter_finish_empty_20261025_234555_682505`는 알려진 760개 목표와 새 마무리 64개 목표, 종료 정지를 모두 완료했다. 최대 추종 0.1058rad, 최대 전류 126mA, 최저 전압 3.8V, 최대 온도 23도다. 실제 읽기 1,033개와 쓰기 이벤트 1,032개를 남겼으며 OFF와 설정 복원을 확인했다. 카메라 1,172장이 모두 유효했다. 카메라 첫 화면 대기도 약 0.5초 뒤 정상 재개됐다.
- 종료 직후 OFF 값은 중력으로 내려오는 중이어서 출발값으로 쓰지 않았다. 이후 `finish_empty_readonly_20261025_235113_620117`에서 세 번 읽은 안정 OFF 값의 최대 차이는 1틱이었다. 이 검토에서는 실제 장치에 읽기만 했으며 목표나 토크 및 설정 쓰기는 0회다. 대표 영상에서 큐브는 보이지 않고 사람 손가락은 본체 쪽에 보인다. 가려진 실제 접촉이 전혀 없었다고 확정하지 않는다.
- 이번 실제 빈손 궤적을 그대로 처방한 별도 큐브 계산에서도 9개 배치가 방향과 유지 및 모델 겹침 기준을 만족했다. 마지막 유지 목표 오차의 최대값은 약 13.65도였으며 손 접촉을 유지했다. 이는 책의 수평과 단단한 접촉, 마찰을 가정한 계산이다. 실물의 큐브 위치 정합과 접촉 강도 및 적재 모터 응답 식별은 아직 아니다. 실제 90도 회전 성공으로 기록하지 않는다.
- 새 `leap_loaded_quarter_finish_trial.cmd`는 빈손으로 완료한 760개 목표와 마무리 64개를 그대로 사용한다. 현재 OFF 자세에서 준비 복귀와 배치 및 제거 확인을 포함하는 전체 손 경로가 정적 201개 조건과 응답 9개 조건을 통과했다. 엄지 ID2의 초기 펴기 읽기가 직전 목표를 20틱 이내로 따르지 못하면 다음 목표를 막는다. 배치 전 목표 전송 금지, 새 구간 전 상태 오류, 제거 미확인과 OFF 및 복원, 증거 위조 거부를 포함한 프로그램 검사 9개도 통과했다. 카메라 첫 화면 수정과 기존 전류 및 PID, 2틱 속도와 실제 건강 상태 중단 기준을 유지했다.
- 다음 사용자 명령은 **`leap_loaded_quarter_finish_trial.cmd`**다. 첫 SPACE는 큐브를 손과 화면 밖에 둔 빈손 시작이다. 준비 복귀 뒤 PLACE에서 검지와 중지의 평평한 모터 및 링크 윗면에 **초록 위, 주황 손목 ID41 방향**으로 놓고 손을 뺀 뒤 둘째 SPACE를 누른다. 기존 38초와 새 3.2초 마무리를 수행한다. REMOVE에서 큐브를 먼저 빼고 셋째 SPACE로 OFF한다. Q와 Esc 및 Ctrl+C는 중단이며 자동 재시도는 없다. 끝나면 했어만 알리면 저장 파일을 검토한다.
- 이번 준비는 첫 후속 적재 실험이며 강건한 적재 제어기나 실물 회전 완료가 아니다. 기존 38초만의 명령을 그대로 반복하도록 요청하지 않는다. 근거는 `finish_empty_completion_2345/summary.json`, `level_book_seating/measured_empty_234555_mu03/seating_grid.json`, `loaded_finish_fixture_v2/readiness_review.json`에 있다. 원본 시퀀스와 범위 JSON, 실제 기록과 전시 문구의 해시는 보존했다.

<!-- END LOADED FINISH READY 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8-sig');marker='<!-- END LOADED FINISH READY 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT 2026-10-25: 빈손 실행 4회','## PREVIOUS 2026-10-25: 빈손 실행 4회',1);p.write_text(text+old,encoding='utf-8')
    require_offline_review()
    print(json.dumps(dict(loaded_finish_trial_ready=True,static_cases=201,native_cases=9,program_tests=9,loaded_goals=824,loaded_motion_seconds=41.2,command=readiness['user_command']),ensure_ascii=False))
if __name__=='__main__':main()
