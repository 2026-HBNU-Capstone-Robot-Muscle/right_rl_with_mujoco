"""Publish the completed run review, study and EMPTY-only next step."""
import datetime as dt,hashlib,json
from pathlib import Path
from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan,ROOT,FIXTURE,proposal_metadata,extra_hashes
from leap_thumb_clear_root_empty_20261026 import require_review

BASE=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454'
def read(p):return json.loads(p.read_bytes())
def write(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    proof=require_review();plan=ThumbRootEmptyPlan(FIXTURE/'request.json');rows=plan.plan(plan.source_position())
    selected=[];sources={}
    for tag in ('thumb_sequential_mu02','thumb_sequential_mu03','thumb_sequential_mu05'):
        f=BASE/tag/'search.json';r=read(f);sources[f.relative_to(ROOT).as_posix()]=sha(f)
        if r.get('hardware_execution_allowed') is not False:raise ValueError('Diagnostic must not authorize loaded hardware')
        for v in r['cases']:
            if v['thumb_delta_ticks']==[0,-128,256,-128] and v['additional_root_ticks']==384:
                if (not v.get('candidate_accepted') or v['order']!='thumb CMC lift -> MCP/axial retreat -> root lift'
                    or v['red_up_error_deg']>10 or v['green_wrist_error_deg']>10):raise ValueError('Selected simulation condition failed')
                selected.append(dict(friction_assumption=r['cube_friction'],**v))
    if len(selected)!=9:raise ValueError('Expected three assumed seats by three friction values')
    poster=ROOT/'exhibition_20261023/족자_붙여넣기.txt'
    baseline=read(ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json')['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    if isinstance(baseline,dict):baseline=baseline['sha256']
    originals={ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
               ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
               Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'):'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656',poster:baseline}
    if any(sha(p)!=expected for p,expected in originals.items()):raise ValueError('Protected original changed')
    stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    study=dict(timestamp=stamp,user_proposal='Move thumb toward wrist and raise index/middle farther',
               source_run='slow_forward_placement_20261026_045454_900706',selected_cases=selected,sources=sources,
               final_face_direction_error_limit_used_deg=10,quaternion_hold_error_limit_used_deg=20,
               max_red_up_error_deg=max(v['red_up_error_deg'] for v in selected),max_green_wrist_error_deg=max(v['green_wrist_error_deg'] for v in selected),
               cube_seating_and_friction_not_identified=True,hand_path_prescribed=True,
               added_comparison_deltas_relative_to_measured_end=True,hardware_targets_relative_to_nominal_end=True,
               study_is_not_a_loaded_hardware_certificate=True,physical_rotation_verified=False,
               assistant_motor_writes=0)
    write(BASE/'proposal_study_review.json',study)
    readiness=dict(timestamp=stamp,ready=True,command='leap_thumb_clear_root_empty.cmd',empty_only=True,
                   source_readonly=read(FIXTURE/'request.json')['readonly_source'],source_readonly_drift_ticks=0,
                   latest_slow_trial_complete=True,latest_slow_trial_min_voltage_v=3.7,
                   latest_slow_trial_peak_current_ma=143,latest_slow_trial_max_tracking_rad=.11658253987930872,
                   power_feed_problem_not_declared_resolved=True,voltage_floor_unchanged=True,
                   proposal=proposal_metadata(),static_conditions=len(proof['static_screens']),native_response_conditions=len(proof['dynamic_sensitivity']),
                   cube_diagnostic_conditions=9,program_tests_passed=9,
                   program_test_source_sha256=sha(ROOT/'test_thumb_clear_root_empty_20261026.py'),
                   prior_jointly_moving_thumb_candidate_rejected_and_archived=True,
                   hardware_health_limits_unchanged=True,collision_detection_and_forces_preserved=True,
                   return_corner_resolution=proof['return_corner_resolution'],
                   return_corner_raw_discarded_depths_mm=proof['return_corner_discarded_maximums_by_front'],
                   planned_goal_count=len(rows),nominal_motion_seconds=len(rows)*plan.SAMPLE_PERIOD,
                   cycle_seconds=plan.SAMPLE_PERIOD,on_timeout_seconds=plan.ON_TIMEOUT,
                   physical_cube_rotation_verified=False,loaded_trial_authorized_by_this_review=False,
                   native_proof_sha256=sha(FIXTURE/'screen.json'),code_hashes=extra_hashes(),
                   preview=(BASE/'preview_sequential/thumb_retreat_then_root_lift.gif').relative_to(ROOT).as_posix(),
                   assistant_motor_goal_writes=0,assistant_torque_changes=0,
                   next_action='EMPTY throughout, Cube outside camera too. Run leap_thumb_clear_root_empty.cmd once, terminal SPACE once. Camera automatic; about2min35s of motion after current-pose screen. New ending lifts thumb, retreats it toward wrist, then raises index/middle by33.75deg. No PLACE prompt or cube. OFF automatic; report done for saved-file review.')
    write(FIXTURE/'readiness_review.json',readiness)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
        p=ROOT/name;r=read(p);r.update(timestamp=stamp,thumb_root_proposal_readiness=readiness,
            next_user_command=readiness['command'],next_user_command_ready=True,next_user_command_empty_only=True,
            next_user_command_photo_only=False,next_user_command_has_loaded_motion=False,rotation_in_next_command=False,
            next_loaded_reference_goal_count=None,next_loaded_motion_seconds=None,
            next_trial_is_guarded_slow_voltage_comparison=False,
            latest_slow_trial_completed_without_voltage_stop=True,power_feed_problem_fixed=False,
            next_action=readiness['next_action'],physical_ninety_degree_rotation_verified=False)
        write(p,r)
    block='''## CURRENT 2026-10-26: 느린 실행 완료 / 사용자 제안의 엄지 후퇴와 검지·중지 추가 세움 준비

- `slow_forward_placement_20261026_045454_900706`는 전체 기준 760개와 기존 마무리 64개 목표를 완료했다. 최저 전압 3.7V, 최고 전류 143mA, 최대 추종 오차 0.11658rad다. 카메라 1,982장이 모두 유효하고 큐브 제거 확인 및 OFF와 설정 복원을 확인했다. 이번에는 전압으로 멈추지 않았지만 공급 경로 문제가 영구 해결됐다고 기록하지 않는다. 실제 90도 회전은 사진만으로 검증되지 않았다.
- 사용자는 엄지를 더 손목 쪽으로 빼고 검지와 중지를 더 세우자고 제안했다. 저장된 실제 관절 경로를 재생한 모델에서는 손목 쪽 가정 배치가 중간 기울기와 엄지 하우징 접촉에 머무는 상황을 재현했다. 엄지의 분리와 후퇴를 먼저 하고 검지와 중지 뿌리를 추가 회전하는 조합을 비교했다. 관절 궤적은 처방하며 큐브의 실제 초기 위치와 마찰 및 접촉은 식별되지 않은 진단이다.
- 첫 동시 엄지 이동 안은 약한 응답 조건에서 약지 끝과 엄지 끝이 최대 약 0.0465mm 겹쳐 폐기했다. 허용치를 늘리지 않고 엄지 CMC를 먼저 128틱 들어 분리한 뒤 MCP -128틱과 축 회전 +256틱으로 손목 방향으로 빼는 순서로 나눴다. 이후 검지와 중지 뿌리를 기존 마무리보다 384틱, 33.75도 더 세운다. 총 엄지 변화는 ID1~4 `[0,-128,+256,-128]`이다. 기존 전류와 PID, 75ms 간격 및 2틱 목표 간격과 전압 및 추종 중단은 유지했다.
- 수정 순서의 큐브 계산은 가정 배치 36/40/44mm와 마찰 0.2/0.3/0.5의 9개 조합을 통과했다. 빨간 위와 초록 손목 방향의 마지막 오차는 각각 최대 약 7.38도와 7.41도다. 실제 큐브 위치에 대한 측정값이나 실물 성공률이 아니다. 하드웨어 목표는 명령 종료 자세 기준이고 진단은 실측 종료 관절 자세 기준이므로 빈손 확인이 우선이다.
- 새 동작 앞의 기존 빈손 복귀에서 약한 응답에 엄지 끝과 손바닥6 이음새가 약 0.00748mm 접촉했다. MCP를 더 펴는 변경안은 다른 모델 간섭을 만들어 미채택했다. 원래 복귀 목표를 보존했고, 이 정확한 쌍에 초기 분리 단계에서 이미 사용하던 0.01mm 모델 해상도를 빈손 복귀 단계에도 명시적으로 적용했다. 5/2.5/1ms 간격에서 원시 접촉은 0.00748/0.00789/0.00816mm로 남았으므로 단순 적분 오차로 사라진다고 주장하지 않는다. 다른 쌍과 다른 단계 또는 그보다 깊은 접촉은 남기고 접촉 힘도 유지한다. 실제 형상 보정이나 무접촉 증명은 아니다.
- 사용자 연결 확인 후 `finish_empty_readonly_20261026_052014_577958`에서 현재 관절값을 세 번 읽었으며 변화는 0틱이다. 이 현재 자세에서 새 빈손 경로의 정적 201개와 응답 9개 조건을 확인했고 프로그램의 경로 순서, 장치 없는 기본 실행, 원래 복귀 보존, 중단 및 건강 상태 실패 시 OFF와 설정 복원, 해상도 처리의 쌍과 단계 제한 및 적재 권한과 목표 위조 거부를 포함한 9개 검사를 통과했다. 초기 네이버후드가 없는 단일 진단의 거부 결과와 실패 개발안은 보존했다.
- 다음 명령은 **`leap_thumb_clear_root_empty.cmd`**다. 처음부터 끝까지 빈손이며 큐브는 카메라 밖에도 둔다. 터미널 SPACE 한 번이고 PLACE 단계는 없다. 현재 자세 검사 후 약 2분35초의 목표 구간을 수행하며 마지막에 엄지 분리와 후퇴 및 검지·중지 추가 세움을 보게 된다. 자동 OFF 뒤 했어만 알려주면 저장 결과를 확인한다. 큐브를 넣는 실행은 아직 준비하지 않았다.
- 근거는 `slow_run_0454/run_review.json`, `proposal_study_review.json`과 `thumb_root_empty_fixture_v5_corner_resolution/readiness_review.json`이다. 미리보기는 `slow_run_0454/preview_sequential/thumb_retreat_then_root_lift.gif`와 `thumb_root_comparison.png`다. 실제 모터 쓰기는 읽기 점검을 포함해 0회이며 원본과 전시 문구는 보존했다.

<!-- END THUMB ROOT USER PROPOSAL CURRENT 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8-sig')
        marker='<!-- END THUMB ROOT USER PROPOSAL CURRENT 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT ','## PREVIOUS ',1)
        p.write_text(block+old,encoding='utf-8')
    print(json.dumps({k:readiness[k] for k in ('ready','command','empty_only','static_conditions','native_response_conditions','program_tests_passed','nominal_motion_seconds','assistant_motor_goal_writes')},ensure_ascii=False))
if __name__=='__main__':main()
