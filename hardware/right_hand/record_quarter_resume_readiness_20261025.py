"""Persist reviewed EMPTY continuation readiness, without touching devices."""
import datetime,hashlib,json
from pathlib import Path
import numpy as np
from reverse_quarter_resume_empty_plan_20261025 import ROOT,GOALS,ReverseQuarterResumeEmptyPlan,resume_goals
from reverse_quarter_resume_empty_gate_20261025 import validate_proof
from goal_aware_progress_guard_20261025 import metadata
OUT=ROOT/'right_hand_rotation_20261024/reverse_quarter_resume_empty'
FIXTURE=OUT/'fixture_v4'

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,obj):Path(path).write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf-8')

def main():
    p=ReverseQuarterResumeEmptyPlan(FIXTURE/'request.json');proof=validate_proof(p,FIXTURE/'screen.json')
    log=(FIXTURE/'test_completion.log').read_text(encoding='utf-8')
    assert 'Ran 28 tests' in log and log.rstrip().endswith('OK')
    baseline=json.loads((ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json').read_bytes())
    poster=ROOT/'exhibition_20261023/족자_붙여넣기.txt'
    assert digest(poster)==baseline['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    reference=Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py')
    assert digest(reference)=='008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'
    assert digest(ROOT/'sequence_flip_poses.json')=='cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b'
    previous=json.loads((ROOT/'right_hand_rotation_20261024/reverse_quarter_path_empty/fixture_v1/screen.json').read_bytes())
    for name,value in {**previous['code_hashes'],**previous['extra_code_hashes']}.items():assert digest(ROOT/name)==value,name
    meta=json.loads((ROOT/GOALS).read_bytes());source=ROOT/meta['source_actual_trial'];actual=json.loads(source.read_bytes())
    assert digest(source)==meta['source_actual_trial_sha256']
    samples=actual['samples'];last=samples[-1];recent=[s for s in samples if s['t']>=last['t']-8]
    failure=dict(source=source.relative_to(ROOT).as_posix(),source_sha256=digest(source),
        error=actual.get('error'),failed_phase=actual.get('failed_phase'),samples=len(samples),
        last_sample_seconds=last['t'],all_torque_off_verified=actual['all_torque_off_verified'],settings_restored=actual['settings_restored'],
        verified_last_issued_goal=last['goal'],verified_last_goal_readback=last['goal_readback'],last_actual=last['actual'],
        legacy_final_command_is_planned_not_actual=True,legacy_planned_final_goal=actual['final_command'],
        id8_last_error_ticks=last['goal'][7]-last['actual'][7],
        id8_last_eight_seconds_actual_span_ticks=int(np.ptp([s['actual'][7] for s in recent])),
        id8_last_eight_seconds_goal_span_ticks=int(np.ptp([s['goal'][7] for s in recent])),
        progress_false_positive_supported=True,physical_contact_absence_verified=False,
        second_attempt='reverse_quarter_path_empty_20261025_082933_354374/result.json',
        second_attempt_writes=0,second_attempt_cause='Mid-path OFF pose failed the old original-start neighborhood check.')
    save(OUT/'interrupted_actual_review.json',failure)
    rows=p.plan(p.source_position());suffix=resume_goals();now=datetime.datetime.now().astimezone().isoformat()
    ready=dict(timestamp=now,user_command='leap_reverse_quarter_resume_empty.cmd',empty_continuation_ready=True,
        cube_allowed=False,cube_placement_checkpoint=False,operator_terminal_space_count=1,torque_off_automatic=True,
        prior_actual_reference_waypoint=meta['resumed_original_waypoint'],prior_executed_reference_seconds=23.6,
        remaining_reference_motion_goals=len(suffix)-1,remaining_reference_motion_seconds=(len(suffix)-1)*.05,
        total_recorded_goals=len(rows),nominal_motor_on_seconds=(len(rows)+20)*.05,
        first_simulation_check_seconds_not_included=True,continuous_whole_reference_execution_verified=False,
        fixture=FIXTURE.relative_to(ROOT).as_posix(),full_static_cases=len(proof['static_screens']),
        full_native_dynamic_cases=len(proof['dynamic_sensitivity']),device_free_tests_passed=28,
        proof_scope='Entire EMPTY recovery from current OFF pose, then exact unexecuted original reference suffix; not the earlier prefix.',
        plan_version=p.PLAN_VERSION,plan_sha256=p.plan_digest(p.source_position()),source_request_sha256=p.SOURCE_SHA,
        progress_policy=metadata(),actual_current_PID_speed_and_health_stops_unchanged=True,
        initial_ring_neighbor_scope='Same ring pad against adjacent fixed palm CAD hull, initial release only, <=1.5mm model bound, no new/deeper contact; zero after release.',
        passive_initial_support_scope='Nominal measured starting pose determines whether a passive-enable pair is new. Counterfactual +/-4-tick membership does not change its classification. Existing .15mm new-pair limit retained.',
        code_hashes=proof['code_hashes'],extra_code_hashes=proof['extra_code_hashes'],
        new_assistant_motor_goal_writes=0,new_assistant_torque_changes=0,
        loaded_cube_execution_ready=False,physical_cube_rotation_verified=False,poster_unchanged=True,original_reference_unchanged=True)
    save(OUT/'readiness_review.json',ready)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json'):
        path=ROOT/name;status=json.loads(path.read_bytes())
        status.update(timestamp=now,next_user_command=ready['user_command'],next_user_command_ready=True,
            next_user_command_empty_only=True,next_user_command_photo_only=False,new_device_free_tests_passed=28,
            source_readonly='reverse_quarter_path_empty_20261025_082933_354374/fresh_off_request.json',
            latest_interrupted_empty_review=failure,empty_continuation_readiness=ready,
            whole_reference_continuously_completed=False,whole_quarter_empty_readiness_current=False,
            next_action='User runs EMPTY continuation once. One terminal SPACE: fresh OFF screening, bounded ring/roots release, join verified last-issued waypoint, remaining 14.4s, automatic OFF. No cube or PLACE.',
            reverse_loaded_execution_ready=False,physical_ninety_degree_rotation_verified=False,
            new_agent_motor_goal_writes=0,new_agent_torque_changes=0)
        save(path,status)
    text='''## CURRENT 2026-10-25: 저속 무진행 오판 수정 / 중단 지점 복귀 후 남은 빈손 경로 실행 준비

- 08:27 실행은 약 32.4초 동안 움직인 뒤 ID8의 기존 무진행 판정으로 중단됐다. ID8은 직전 8초 동안 실제 23틱, 목표 28틱 변화가 있어 계속 막혀 있던 기록이 아니다. 마지막 약 1초 목표 변화는 4틱뿐이었다. 중단 후 OFF 확인과 설정 복원은 정상이었다. 실물 접촉이 전혀 없었다고 확정한 것은 아니다. 08:29 재실행은 중간 자세를 원래 시작 근처로 인정하지 않아 쓰기 없이 거부됐다.
- 새 독립 실행 파일은 목표의 의미 있는 이동과 관절의 실제 이동을 함께 보는 판정을 사용한다. 실제 639개 읽기를 재생하면 잘못 멈추지 않으며, 목표가 계속 변하는데 실제 관절이 막힌 검사에서는 멈춘다. 기존 실제 추종 0.18rad와 전류 및 전압 중단, 전류와 PID, 2틱/50ms 제한은 유지했다. 공용 기존 실행 코드는 수정하지 않았다.
- 다음 사용자 작업은 **`leap_reverse_quarter_resume_empty.cmd`**다. 큐브는 끝까지 손과 화면 밖, 손가락과 받침은 그대로 둔다. 카메라는 자동으로 열리며 터미널 SPACE 한 번으로 현재 OFF 측정과 검사를 수행한다. 약지를 조금 펴고 검지와 중지 뿌리를 먼저 올린 뒤 실제 마지막 목표로 복귀하여 남은 14.4초만 진행하고 자동 OFF한다. 현재 입력의 명목 ON 시간은 약 23.9초이며 검사와 카메라 시작 시간은 별도다. PLACE나 추가 SPACE는 없다. 끝나면 했어만 알리면 저장 기록을 검토한다.
- 복귀 기준은 이전 결과의 `final_command`가 아니라 마지막 실제 읽기에서 목표와 readback이 같은 472번 지점이다. 이전 `final_command`는 아직 보내지 않은 계획 끝점이었다. 새 실행 기록은 `planned_final_command`와 `last_issued_command`를 나누며 `final_command`도 실제 마지막 전송값으로 남긴다. 앞부분 23.6초를 반복하지 않는다. 두 실행으로 나눠 확인하는 것으로 전체 연속 실행을 완료했다고 표시하지 않는다.
- `reverse_quarter_resume_empty/fixture_v4`에서 복귀와 남은 구간 전체의 201개 정적 조건과 9개 동역학 조건, 독립 Windows 검증 및 장치 없는 28개 검사를 통과했다. 초기 약지 패드의 옆 손바닥 도형 접촉은 이번 복귀에만 1.5mm 모델 상한 안에서 깊어지지 않고 분리하도록 검사하며 분리 후 자기 겹침은 0이다. 실제 침투량 측정이 아니다. 전원 정착 중 새 받침 접촉의 기존 0.15mm 기준은 유지하고, 실제 시작점과 가상의 +/-4틱 상태를 혼동한 분류 오류를 수정했다. 실패한 개발 검사도 보존했다.
- 큐브를 올린 90도 회전은 여전히 미검증이다. 이번에는 실제 장치를 열거나 모터 쓰기 및 토크 변경을 하지 않았다. 큐브 배치 사진을 다시 요청하지 않는다. 원본 동작 파일과 과거 실제 기록 및 전시 문구는 변경하지 않았다.

<!-- END EMPTY QUARTER RESUME 20261025 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith('## CURRENT 2026-10-25: 저속 무진행 오판 수정'):
            old=old.split('<!-- END EMPTY QUARTER RESUME 20261025 -->',1)[1].lstrip()
        path.write_text(text+old.replace('## CURRENT','## PREVIOUS'),encoding='utf-8')
    (OUT/'README.md').write_text('''# 중단된 빈손 90도 후보 경로 이어가기

실행은 `leap_reverse_quarter_resume_empty.cmd`다. 큐브를 손과 화면 밖에 두고 손가락과 받침은 그대로 둔다. 카메라는 자동으로 열린다. 터미널 SPACE 한 번으로 새 OFF 값을 읽고 검사한 뒤 약지 접촉 해제와 뿌리 이동 및 중단 지점 복귀를 수행한다. 남은 14.4초를 한 번 움직이고 자동 OFF한다. 총 ON 명목 시간은 약 23.9초이며 모델 검사 시간은 별도다. PLACE나 추가 SPACE는 없다. Q 또는 Esc 및 Ctrl+C로 중단할 수 있다. 끝나면 했어만 알려주면 된다.

이전 08:27 실행은 저속 목표를 무진행으로 오판했다. 마지막 실제 읽기와 readback이 일치하는 원본 472번 지점에서 이어가며 이미 진행한 23.6초는 반복하지 않는다. 예전 기록의 final_command는 계획 끝점이므로 복귀에 사용하지 않는다. 원본과 실제 결과는 보존했다.

새 판정은 같은 방향 목표 이동이 16틱 이상이고 충분한 읽기와 시간이 쌓였는데 실제 진행이 2틱 이내이며 방향 오차가 64틱 이상일 때 멈춘다. 실제 건강 검사와 전류 및 PID, 목표 속도 제한은 그대로다. 고정 목표의 정적 오차를 이동 요청으로 취급하지 않는다. 느린 목표에 대한 판정 범위와 한계는 progress_policy에 기록했다.

fixture_v4는 최신 저장 OFF 상태의 전체 복귀와 남은 구간을 검사한 개발 입력이다. 201개 정적 조건과 9개 동역학 조건, 독립 검증과 장치 없는 검사 28개를 통과했다. 실행 때 새 현재 값을 다시 읽고 검증하므로 이 개발 입력으로 실물을 대신하지 않는다. fixture_v1 실패와 후속 개발 결과를 남겼다. 증거 파일의 검사 범위는 복귀와 미실행 후반 구간이며 전체 38초의 연속 실물 실행이 아니다.

실제 큐브 회전이나 라이브 SAC 실행이 아니다. 실제 90도 성공과 적재 실행 준비는 여전히 false다. 배치 사진은 확보되어 반복 요청하지 않는다. 전시 문구는 수정하지 않았다.
''',encoding='utf-8')
    print(json.dumps({k:ready[k] for k in ('empty_continuation_ready','user_command','remaining_reference_motion_seconds','nominal_motor_on_seconds','device_free_tests_passed','loaded_cube_execution_ready','poster_unchanged')},ensure_ascii=False))

if __name__=='__main__':main()
