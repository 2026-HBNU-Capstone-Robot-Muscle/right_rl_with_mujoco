"""Record a guarded slower trial under the user's no-rewiring preference."""
import datetime as dt,hashlib,json
from pathlib import Path
import numpy as np
from slow_forward_plan_20261026 import SlowForwardPlan,ROOT,FIXTURE,extra_hashes,TURN_PHASE,FINISH_PHASE
from recovered_forward_plan_20261026 import RecoveredForwardPlan
from leap_slow_forward_placement_20261026 import require_review
from thumb_mcp_clearance_guard_20261026 import policy as release_policy

def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    proof=require_review();plan=SlowForwardPlan(FIXTURE/'request.json');raw=plan.source_position();rows=plan.plan(raw)
    old=RecoveredForwardPlan.__new__(RecoveredForwardPlan);old.SOURCE=plan.SOURCE;old.SOURCE_SHA=plan.SOURCE_SHA;old.center=plan.center.copy()
    fast=old.plan(raw)
    if [p for p,q in rows]!=[p for p,q in fast] or not np.array_equal([q for p,q in rows],[q for p,q in fast]):raise ValueError('Slow trial changed motor goal positions/order')
    originals={ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
        ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
        Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'):'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'}
    if any(sha(p)!=v for p,v in originals.items()):raise ValueError('Original changed')
    poster=ROOT/'exhibition_20261023/족자_붙여넣기.txt';base=read(ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json')['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    if isinstance(base,dict):base=base['sha256']
    if sha(poster)!=base:raise ValueError('Poster changed')
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    r=dict(timestamp=stamp,ready=True,user_command='leap_slow_forward_placement.cmd',user_declines_rewiring=True,
        wiring_photo_not_required_for_this_experiment=True,operator_confirmed_cube_removed=True,
        source_readonly='finish_empty_readonly_20261026_043436_613651/result.json',source_readonly_maximum_drift_ticks=1,
        period_before_s=.05,period_after_s=.075,motion_duration_ratio=1.5,on_timeout_s=180,
        goal_positions_and_order_unchanged=True,total_goal_count=len(rows),nominal_motion_seconds=len(rows)*.075,
        loaded_reference_goals=sum(p==TURN_PHASE for p,q in rows),loaded_finish_goals=sum(p==FINISH_PHASE for p,q in rows),
        nominal_loaded_turn_seconds=61.8,voltage_floor_lowered=False,
        hard_35v_stop_unchanged=True,original_voltage_recovery_policy_unchanged=True,
        PID_current_native_ranges_tracking_and_camera_stops_unchanged=True,
        voltage_improvement_verified=False,electrical_supply_model_identified=False,power_feed_problem_resolved=False,
        slow_seam_resolution=proof['slow_thumb_seam_resolution'],slow_raw_discarded_depths_mm=proof['slow_thumb_seam_discarded_maximums_by_front'],
        simulation_step_s=.005,clearance_contact_forces_disabled=False,
        native_proof=FIXTURE.relative_to(ROOT).as_posix()+'/screen.json',native_proof_sha256=sha(FIXTURE/'screen.json'),
        code_hashes=extra_hashes(),static_cases=len(proof['static_screens']),native_response_cases=len(proof['dynamic_sensitivity']),
        program_tests_passed=9,tests=dict(flow_and_voltage_floor=6,numerical_resolution_scope=2,timing_proof_tampering=1),
        program_test_source_sha256=sha(ROOT/'test_slow_forward_20261026.py'),
        mcp_release_policy=release_policy(plan,proof),physical_cube_rotation_verified=False,
        assistant_motor_goal_writes=0,assistant_torque_changes=0,
        next_action='Use leap_slow_forward_placement.cmd once, EMPTY at first SPACE. At PLACE use the same GREEN up/ORANGE wrist and about5mm forward placement; hands clear then second SPACE. Slower turn about62s plus observation. REMOVE Cube before third SPACE/OFF. Voltage stops remain active; report done for local review.')
    write(FIXTURE/'readiness_review.json',r)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        p=ROOT/name;v=read(p);v.update(timestamp=stamp,slow_forward_readiness=r,
            user_declines_rewiring=True,power_hub_photo_request_cancelled=True,power_feed_problem_fixed=False,
            power_feed_review_required_before_next_loaded_trial=False,next_trial_is_guarded_slow_voltage_comparison=True,
            next_user_command=r['user_command'],next_user_command_ready=True,next_user_command_empty_only=False,
            next_user_command_photo_only=False,next_user_command_has_loaded_motion=True,rotation_in_next_command=True,
            next_loaded_reference_goal_count=824,next_loaded_motion_seconds=61.8,next_action=r['next_action'],physical_ninety_degree_rotation_verified=False)
        write(p,v)
    block='''## CURRENT 2026-10-26: 배선 유지 / 전압 하한 유지 / 1.5배 느린 실행 준비

- 사용자는 연결 변경을 원하지 않으며, 문제가 없다면 전압 하한을 낮춰 달라고 했다. XC330-M288 공식 입력은 3.7~6.0V이므로 3.5V 아래 운전이 문제없다고 확인할 수 없다. 하한과 기존 전압 회복 로직은 바꾸지 않았다. 전원 허브 사진 요청은 취소하고 같은 배선에서 느린 동작을 비교한다. 전압 문제가 해결됐거나 더 낮은 전압이 안전하다고 기록하지 않는다.
- 사용자가 큐브 제거를 확인한 뒤 `finish_empty_readonly_20261026_043436_613651`에서 관절을 세 번 읽었다. 최대 차이는 1틱이며 목표와 토크 쓰기는 0회다. 새 모드는 같은 목표 자세와 순서를 50ms가 아닌 75ms 간격으로 보낸다. 기존 38초 기준 구간은 57초, 마무리 3.2초는 4.8초가 된다. 적재 회전은 합계 약 61.8초와 종료 관찰이다. 전체 계획 1,526개 목표의 명목 시간은 114.45초이며 구동 제한은 180초로 맞췄다. 사람의 배치와 제거 대기는 별도다.
- 75ms를 10ms 계산 간격에 맞춘 첫 계산에서는 약한 응답에서 엄지 끝과 손바닥7 모서리에 0.00485mm 접촉이 나왔다. 5ms로 정밀 계산하면 약 0.00109mm였다. 같은 모서리에서 이미 사용하던 0.01mm 수치 분류를 느린 모드의 이후 자세에도 명시적으로 적용하고 버린 원시 깊이를 기록했다. 다른 접촉과 0.01mm보다 큰 접촉은 남겨 검사하며 실제 접촉 힘이나 메시를 제거하지 않았다. 이 숫자를 실물 접촉 보정이나 원시 무접촉 증명으로 기록하지 않는다. 이전 실패 계산은 보존했다.
- 최종 5ms 계산에서 현재 자세의 정적 경로 201개와 응답 9개가 통과했다. 프로그램은 동일한 목표 순서, 배치와 제거 및 취소, 건강 상태 오류의 목표 금지, 3.5V 중단 유지, 장치 없는 기본 실행, 수치 분류 범위와 시간 증거 변조 거부를 포함한 9개 검사를 통과했다. 전기 공급 모델은 없으므로 느리게 하면 전압이 개선된다는 검증은 아니다. 실제 전류와 PID, 관절 범위, 0.18rad 추종과 카메라 중단은 유지한다.
- 다음 명령은 **`leap_slow_forward_placement.cmd`**다. 빈손으로 첫 SPACE, PLACE 이후 같은 초록 위와 주황 손목 방향 및 약 5mm 손끝 쪽 배치를 사용하고 손을 뺀 뒤 둘째 SPACE다. 약 62초의 느린 회전과 관찰 후 REMOVE에서 큐브를 먼저 빼고 셋째 SPACE로 OFF한다. 다시 3.5V라면 중단하며 실제 전압 회복과 회전은 결과 파일로 확인한다. 배선을 바꾸거나 전원장치 출력을 올리도록 요청하지 않는다.
- 근거는 `loaded_finish_review_0057_20261026/slow_forward_fixture_v2_seam_resolution/readiness_review.json`이다. 실제 90도 성공은 미검증이며 원본, 실제 기록, 범위 JSON과 전시 문구는 보존했다.

<!-- END SLOW VOLTAGE COMPARISON CURRENT 20261026 -->

'''
    marker='<!-- END SLOW VOLTAGE COMPARISON CURRENT 20261026 -->'
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8-sig')
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT 2026-10-26: 큐브 적재 회전 중 3.5V 관측 / 전원 경로 확인 필요','## PREVIOUS 2026-10-26: 큐브 적재 회전 중 3.5V 관측 / 전원 경로 확인 필요',1)
        p.write_text(block+old,encoding='utf-8')
    print(json.dumps(dict(ready=True,command=r['user_command'],loaded_seconds=61.8,static_cases=201,response_cases=9,
        program_tests=9,voltage_floor_lowered=False,voltage_improvement_verified=False,assistant_motor_writes=0),ensure_ascii=False))
if __name__=='__main__':main()
