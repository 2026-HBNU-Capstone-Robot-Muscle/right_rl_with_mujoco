"""Record why repeated runs stopped and the validated current-posture recovery."""
import datetime as dt,hashlib,json
from pathlib import Path
import numpy as np
from recovered_forward_plan_20261026 import ROOT,FIXTURE,RecoveredForwardPlan,extra_hashes,TURN_PHASE,FINISH_PHASE
from forward_cube_placement_20261026 import ForwardPlacementPlan
from interrupted_thumb_contact_policy_20261026 import metadata
from leap_recovered_forward_placement_20261026 import require_review

OUT=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026'
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def main():
    proof=require_review();plan=RecoveredForwardPlan(FIXTURE/'request.json');raw=plan.source_position()
    parent=ForwardPlacementPlan.__new__(ForwardPlacementPlan);parent.SOURCE=plan.SOURCE;parent.SOURCE_SHA=plan.SOURCE_SHA;parent.center=plan.center.copy()
    rows=plan.plan(raw);old=parent.plan(raw)
    if [p for p,q in rows]!=[p for p,q in old] or not np.array_equal([q for p,q in rows],[q for p,q in old]):raise ValueError('Recovery changed target arrays')
    protected={ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
        ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
        Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'):'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'}
    if any(sha(p)!=value for p,value in protected.items()):raise ValueError('Original modified')
    baseline=read(ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json')['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    if isinstance(baseline,dict):baseline=baseline['sha256']
    if sha(ROOT/'exhibition_20261023/족자_붙여넣기.txt')!=baseline:raise ValueError('Poster modified')
    camera=ROOT/'blank_camera_probe_20261026_032729_565099/result.json';cam=read(camera)
    failures=read(OUT/'forward_repeated_stop_review.json');new=[v for v in failures if v['folder'].startswith('forward_cube_placement_')]
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    r=dict(timestamp=stamp,user_command='leap_recovered_forward_placement.cmd',ready=True,
        recent_saved_comparison_attempts=len(new),camera_failure_attempts=sum('Camera' in (v.get('error') or '') for v in new),
        interrupted_off_contact_rejection_attempts=sum('OFF initial casing/ring contact' in (v.get('error') or '') for v in new),
        all_recent_comparison_attempts_before_place=True,physical_ninety_degree_rotation_verified=False,
        source_readonly='finish_empty_readonly_20261026_032730_027771/result.json',source_readonly_maximum_drift_ticks=1,
        code_hashes=extra_hashes(),proof_sha256=sha(FIXTURE/'screen.json'),proof=FIXTURE.relative_to(ROOT).as_posix()+'/screen.json',
        static_cases=len(proof['static_screens']),dynamic_cases=len(proof['dynamic_sensitivity']),program_tests_passed=6,
        program_test_source_sha256=sha(ROOT/'test_recovered_forward_20261026.py'),
        initial_contact_policy=metadata(raw),initial_overall_thumb_envelope_unchanged_mm=4.,
        goal_arrays_identical_to_previous_comparison=True,total_goal_count=len(rows),
        loaded_reference_goals=sum(p==TURN_PHASE for p,q in rows),loaded_finish_goals=sum(p==FINISH_PHASE for p,q in rows),
        physical_contact_mesh_calibration_verified=False,cube_contact_in_native_hand_screen=False,
        actual_camera_blank_stop_unchanged_seconds=.5,actual_motor_health_stops_unchanged=True,
        standalone_camera_probe=dict(path=camera.relative_to(ROOT).as_posix(),sha256=sha(camera),
            elapsed_s=cam['elapsed_s'],usable_frames=cam['usable'],blank_frames=cam['blank'],
            intermittent_black_source_root_cause_identified=False,motor_commands_sent=0),
        assistant_servo_write_commands_sent=0,assistant_torque_changes=0,
        next_action='Run new command once from unchanged EMPTY hand. At PLACE, GREEN up, ORANGE toward wrist, about5mm farther toward tips than earlier placement. Hands clear then second SPACE; REMOVE before third SPACE. If camera goes black, do not repeatedly restart; check camera connection. No automatic motion retry.')
    write(FIXTURE/'readiness_review.json',r)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json',
            'right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json',
            'right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        p=ROOT/name;v=read(p);v.update(timestamp=stamp,repeated_comparison_stop_review=r,
            next_user_command=r['user_command'],next_user_command_ready=True,next_user_command_empty_only=False,
            next_user_command_photo_only=False,next_user_command_has_loaded_motion=True,rotation_in_next_command=True,
            next_loaded_reference_goal_count=824,next_loaded_motion_seconds=41.2,next_action=r['next_action'],
            physical_ninety_degree_rotation_verified=False)
        if isinstance(v.get('placement_comparison_readiness'),dict):v['placement_comparison_readiness'].update(superseded_next_command=r['user_command'],current_off_recovery_rejected_in_actual_runs=True)
        write(p,v)
    block='''## CURRENT 2026-10-26: 카메라 중단 뒤 자세의 복귀 검사 보완 / 실물 회전은 미검증

- 새 배치 비교 저장 결과 4개를 확인했다. 03:12 시도는 OFF 검사 중 검은 카메라 입력이 10초 안에 복구되지 않아 쓰기 없이 종료됐다. 03:14 시도는 경로 검사를 통과한 뒤 빈손 복귀 중 카메라가 0값 화면을 보내 중단됐다. 이 시도는 쓰기 이벤트 361개 뒤 모든 OFF와 설정 복원을 확인했다. 03:17과 03:19는 그때 OFF로 내려온 자세의 초기 엄지 접촉 검사에서 쓰기 없이 거부됐다. 네 번 모두 PLACE 이전이며 적재 회전 비교는 실행되지 않았다.
- 현재 안정 OFF 값은 `finish_empty_readonly_20261026_032730_027771`에서 세 번 읽었고 최대 차이는 1틱이다. 진단 모델에서 엄지 끝이 손바닥의 두 인접 영역과 닿는 자세였다. 이전 시작 검사는 한 영역에 0.5mm만 허용했고 다른 영역은 초기 허용 집합에서 빠져 있었다. 이는 측정된 실제 2mm 압축이나 실제 충돌 확정이 아니다.
- 새 처리는 이 기록 자세의 16틱 주변에만 적용한다. 기존 전체 엄지 초기 범위 4mm 안에서 두 영역을 초기 접촉으로 분류하고, 정적 경로에서는 단조 분리, 응답 계산에서는 초기 접촉 범위 초과 금지를 요구한다. 분리 뒤 겹침은 계속 금지한다. 손 메시와 힘, 실제 전류와 PID, 관절 범위, 카메라 0.5초 검은 화면 중단과 실제 모터 중단 기준은 바꾸지 않았다. 처음 2.5mm 후보는 시작 감도 조건을 통과하지 못했고, 두 영역의 분류가 완성되지 않은 계산도 거부됐다. 그 거부 기록은 보존했다.
- 최종 경로는 정적 201개와 응답 9개를 통과했고, 초기 정책의 범위와 목표 동일성 및 배치 취소와 OFF, 검증 데이터 변조 거부를 포함한 프로그램 검사 6개가 통과했다. 모터 목표와 각 구간은 이전 배치 비교와 같다. 카메라만 40초 확인한 782장은 모두 정상이었지만 간헐적인 0값 입력의 원인은 아직 재현하거나 식별하지 못했다.
- 다음 명령은 **`leap_recovered_forward_placement.cmd`**다. 현재 손을 바꾸지 말고 큐브를 빼둔 상태로 첫 SPACE를 누른다. PLACE에서 이전보다 손끝 쪽으로 약 5mm, 초록 위와 주황 손목 방향으로 놓고 손을 뺀 뒤 둘째 SPACE다. REMOVE에서 먼저 빼고 셋째 SPACE로 OFF한다. 카메라가 다시 검게 되면 같은 실행을 반복하지 말고 카메라 연결부터 확인한다. 기존 `leap_forward_cube_placement.cmd`는 이번 OFF 자세의 초기 접촉을 처리하지 못하므로 교체된 명령을 사용한다.
- 근거는 `loaded_finish_review_0057_20261026/forward_repeated_stop_review.json`, `interrupted_off_contact_diagnostic.json`, `recovered_forward_fixture/readiness_review.json`이다. 실제 장치에는 읽기와 카메라 입력 확인만 했으며 이 수정 작업의 목표 및 토크 쓰기는 0회다. 원본과 실제 기록, 전시 문구는 보존했다. 실물 90도 완료로 기록하지 않는다.

<!-- END RECOVERED FORWARD CURRENT 20261026 -->

'''
    marker='<!-- END RECOVERED FORWARD CURRENT 20261026 -->'
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8-sig')
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT 2026-10-26: 적재 동작 완료, 실제 넘김은 미완료 / 배치 5mm 비교 준비','## PREVIOUS 2026-10-26: 적재 동작 완료, 실제 넘김은 미완료 / 배치 5mm 비교 준비',1)
        p.write_text(block+old,encoding='utf-8')
    print(json.dumps(dict(ready=True,command=r['user_command'],static_cases=201,response_cases=9,program_tests=6,
        camera_40s_blank_frames=0,motor_goals_changed=False,assistant_motor_writes=0),ensure_ascii=False))
if __name__=='__main__':main()
