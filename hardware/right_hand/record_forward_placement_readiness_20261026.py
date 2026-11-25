"""Publish only the placement comparison; preserve all physical trial evidence."""
import json
import datetime as dt
from pathlib import Path
from forward_cube_placement_20261026 import (
    ROOT, SOURCE, SOURCE_SHA, REVIEW, FILES, sha, ForwardPlacementPlan, require_placement_review)
from loaded_quarter_finish_gate_20261025 import validate_proof
from loaded_quarter_finish_plan_20261025 import LoadedQuarterFinishPlan, TURN_PHASE, FINISH_PHASE
from prepare_forward_placement_review_20261026 import CURRENT_FIXTURE, READONLY

def read(p):return json.loads(p.read_bytes())
def write(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def main():
    plan=ForwardPlacementPlan(CURRENT_FIXTURE/'request.json')
    proof=validate_proof(plan,CURRENT_FIXTURE/'screen.json')
    base=LoadedQuarterFinishPlan(CURRENT_FIXTURE/'request.json')
    if plan.plan_digest(plan.source_position())!=base.plan_digest(base.source_position()):
        raise ValueError('Placement-only route changed')
    originals={ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
        ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
        Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'):'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656',
        SOURCE:SOURCE_SHA}
    for p,value in originals.items():
        if sha(p)!=value:raise ValueError('Original/source changed: '+str(p))
    poster=ROOT/'exhibition_20261023/족자_붙여넣기.txt'
    baseline=read(ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json')['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    if isinstance(baseline,dict):baseline=baseline['sha256']
    if sha(poster)!=baseline:raise ValueError('Poster changed')
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    rows=plan.plan(plan.source_position())
    r=read(REVIEW);r.update(timestamp=stamp,review_ready=True,
        mode_code_hashes={n:sha(ROOT/n) for n in FILES},
        user_command='leap_forward_cube_placement.cmd',
        program_tests_passed=12,program_test_source_sha256=sha(ROOT/'test_forward_cube_placement_20261026.py'),
        guard_rejection_test_rerun_after_strengthening_passed=True,
        current_readonly_source=READONLY.relative_to(ROOT).as_posix(),
        current_readonly_sha256=sha(READONLY),current_readonly_maximum_drift_ticks=1,
        current_native_proof=(CURRENT_FIXTURE/'screen.json').relative_to(ROOT).as_posix(),
        current_native_proof_sha256=sha(CURRENT_FIXTURE/'screen.json'),
        current_native_static_cases=len(proof['static_screens']),
        current_native_dynamic_cases=len(proof['dynamic_sensitivity']),
        current_native_proof_includes_cube=False,
        route_plan_digest=plan.plan_digest(plan.source_position()),
        same_goal_arrays_as_parent=True,total_goal_count=len(rows),
        loaded_reference_goals=sum(p==TURN_PHASE for p,q in rows),
        loaded_finish_goals=sum(p==FINISH_PHASE for p,q in rows),
        no_extra_thumb_or_root_targets=True,PID_current_rate_and_health_checks_unchanged=True,
        runtime_fresh_off_screen_still_required=True,automatic_motion_retry=False,live_SAC_policy=False,
        original_and_poster_hashes_unchanged=True,assistant_servo_write_commands_sent=0,
        next_action='EMPTY at first SPACE. At PLACE, shift initial Cube about5mm toward fingertips from previous placement, on flat index/middle tops. GREEN up, ORANGE toward wrist/ID41. Hands clear, second SPACE runs the same824 loaded goals. REMOVE before third SPACE/OFF. Q if unsupported. One placement comparison; physical outcome remains unverified.')
    write(REVIEW,r)
    try:require_placement_review()
    except Exception:
        r['review_ready']=False;write(REVIEW,r);raise
    outcome=read(REVIEW.parent/'physical_partial_review.json')
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json',
            'right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json',
            'right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        p=ROOT/name;v=read(p)
        if isinstance(v.get('loaded_finish_readiness'),dict):
            v['loaded_finish_readiness'].update(supervised_experimental_loaded_trial_ready=False,
                trial_executed=True,physical_cube_rotation_verified=False,
                subsequent_outcome='Tilted; user suspects thumb obstruction',
                superseded_next_action='leap_forward_cube_placement.cmd')
        v.update(timestamp=stamp,latest_loaded_finish_outcome=outcome,
            physical_ninety_degree_rotation_verified=False,
            next_user_command=r['user_command'],next_user_command_ready=True,
            next_user_command_empty_only=False,next_user_command_photo_only=False,
            next_user_command_has_loaded_motion=True,rotation_in_next_command=True,
            next_loaded_reference_goal_count=824,next_loaded_motion_seconds=41.2,
            next_action=r['next_action'],placement_comparison_readiness=r,
            unchanged_placement_repeat_requested=False,supervised_experimental_loaded_trial_ready=True)
        if name.endswith('loaded_quarter_trial_20261025/readiness_review.json'):
            v['supervised_experimental_loaded_trial_ready']=False
            v['new_loaded_finish_trial_ready']=False
            v['new_placement_comparison_ready']=True
        write(p,v)
    block='''## CURRENT 2026-10-26: 적재 동작 완료, 실제 넘김은 미완료 / 배치 5mm 비교 준비

- `loaded_quarter_finish_trial_20261026_005744_776918`는 760개 기준 목표와 64개 마무리 목표를 완료했다. 최대 추종 오차 0.1104rad, 최대 전류 127mA, 최저 전압 3.8V였다. 읽기 2,343개와 쓰기 이벤트 1,637개, 유효 카메라 화면 1,874장을 저장했다. 큐브 제거 확인과 모든 토크 OFF, 설정 복원도 기록됐다.
- 사용자는 큐브가 기울어진 채 멈췄고 엄지에 걸려 넘어가지 못한 것 같다고 보고했다. **실물 90도 회전은 미완료다.** 정확히 엄지의 어느 부위에 닿았는지는 식별하지 않았다. 이미지에서 새 윗면이 가려져 있어 신뢰할 수 있는 실제 회전각도 계산도 확보하지 못했다.
- 실제 적재 관절 궤적을 넣은 접촉 계산에서 손목 가까운 초기 배치 4개는 모두 기울어진 채 남았고, 손끝 쪽 초기 배치 3개는 목표 방향에 가까워졌다. 엄지 부근 접촉이 남는 조건도 있었다. 이 계산은 책의 수평, 단단한 접촉과 마찰, 처방된 손 운동을 가정한다. 실제 배치의 좌표 정합은 없으므로 실물 원인 확정이나 마찰 식별로 기록하지 않는다. 추가 엄지 및 뿌리 운동 후보 66개는 완료 방향과 유지 기준을 함께 통과하지 못해 실물에 설치하지 않았다.
- 현재 OFF 자세는 별도 읽기 전용 결과 `finish_empty_readonly_20261026_012502_138255`에서 세 번 읽었고 최대 차이 1틱이었다. 목표 및 토크 쓰기는 0회다. 현재 자세에서 같은 손 경로를 다시 검사한 정적 201개와 응답 9개 조건이 통과했다. 배치 확인 전 적재 목표 금지, 배치 취소와 제거 미확인 처리, OFF와 복원을 포함해 변경된 배치 절차의 프로그램 검사 12개가 통과했다. 해당 손 경로 검사는 큐브 접촉의 실물 검증이 아니다.
- 다음은 **`leap_forward_cube_placement.cmd`**로 초기 큐브 위치만 비교한다. 이전 위치보다 손끝 쪽으로 약 5mm, 엄지와 손목에서 멀어지게 놓는다. 초록 위, 주황 손목 ID41 방향을 유지하고 검지와 중지의 평평한 커버 위에서 자연히 받쳐져야 한다. 화살표 사진은 `loaded_finish_review_0057_20261026/forward_placement_guide.html`에 있다. 표시 길이나 점선은 좌표 정합된 실측값이 아니다.
- 첫 SPACE는 빈손이다. 준비 후 PLACE에서 새 배치로 놓고 사람 손을 뺀 뒤 둘째 SPACE를 누른다. 같은 824개 목표를 한 번 실행한다. REMOVE에서 큐브를 먼저 빼고 셋째 SPACE로 OFF한다. 받쳐지지 않으면 Q다. 새 엄지 및 뿌리 목표, 전류나 PID 증가, 실제 건강 상태 중단 기준 변경은 없다. 실행 시에는 현재 OFF 값을 다시 읽고 경로를 검사한다. 자동 재시도와 실시간 SAC는 없다. 원래 배치를 그대로 반복하도록 요청하지 않는다.
- 근거는 `loaded_finish_review_0057_20261026/physical_partial_review.json`과 `forward_placement_readiness.json` 및 `forward_placement_current_off_fixture/screen.json`에 있다. 원본 시퀀스와 범위, 실제 기록과 전시 문구는 보존했다.

<!-- END FORWARD PLACEMENT CURRENT 20261026 -->

'''
    marker='<!-- END FORWARD PLACEMENT CURRENT 20261026 -->'
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8-sig')
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT 2026-10-26: 새 마무리 포함 빈손 완료 / 큐브를 올리는 후속 실험 준비',
            '## PREVIOUS 2026-10-26: 새 마무리 포함 빈손 완료 / 큐브를 올리는 후속 실험 준비',1)
        p.write_text(block+old,encoding='utf-8')
    print(json.dumps(dict(placement_comparison_ready=True,user_command=r['user_command'],
        native_static_cases=201,native_response_cases=9,program_tests=12,
        same_loaded_goals=824,prior_physical_rotation_complete=False,
        assistant_servo_write_commands_sent=0),ensure_ascii=False))

if __name__=='__main__':main()
