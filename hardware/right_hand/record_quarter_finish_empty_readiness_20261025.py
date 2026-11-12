"""Record a concrete next EMPTY test, without reissuing loaded approval."""
import hashlib,json
from datetime import datetime
from pathlib import Path
from quarter_finish_empty_plan_20261025 import QuarterFinishEmptyPlan,ROOT,FIXTURE,extra_hashes,finish_goals
from quarter_finish_empty_gate_20261025 import validate_proof
def read(p):return json.loads((ROOT/p).read_bytes())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,r):(ROOT/p).write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    plan=QuarterFinishEmptyPlan(FIXTURE/'request.json');proof=validate_proof(plan,FIXTURE/'screen.json');stamp=datetime.now().astimezone().isoformat();reviewdir='right_hand_rotation_20261024/support_photo_review_221703';base=ROOT/reviewdir/'level_book_seating'
    stats=[]
    for friction in (.2,.3,.5):
        p=base/f'whole_finish128_dt0.0010_tc0.004_mu{friction}'/'seating_grid.json';r=json.loads(p.read_bytes());cases=r['cases']
        stats.append(dict(cube_friction=friction,cases=len(cases),orientation_retention_passes=sum(s.get('orientation_retention_pass',False) for s in cases),candidate_geometry_passes=sum(s.get('candidate_accepted',False) for s in cases),
            max_target_error_deg=max(s['hold_error_deg'] for s in cases),max_red_up_error_deg=max(s['red_up_error_deg'] for s in cases),max_green_wrist_error_deg=max(s['green_wrist_error_deg'] for s in cases),max_cube_model_overlap_mm=max(s['max_cube_overlap_mm'] for s in cases),max_no_hand_contact_s=max(s['max_no_hand_contact_s'] for s in cases),source=p.relative_to(ROOT).as_posix(),source_sha256=sha(p)))
    if sum(s['candidate_geometry_passes'] for s in stats)!=27:raise ValueError('Conditional candidate comparisons incomplete')
    protected={
        ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
        ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
        Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'):'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'}
    hashes={str(p):sha(p) for p in protected}
    if any(hashes[str(p)]!=h for p,h in protected.items()):raise ValueError('Protected original changed')
    poster=ROOT/'exhibition_20261023/족자_붙여넣기.txt';baseline=read('right_hand_rotation_20261024/original_sequence_review/review_summary.json')['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    if isinstance(baseline,dict):baseline=baseline['sha256']
    if sha(poster)!=baseline:raise ValueError('Poster changed')
    rows=plan.plan(plan.source_position());report=dict(timestamp=stamp,conditional_comparisons=stats,
        placement_grid_book_frame_mm=dict(forward=[42,44,46],lateral=[12,15,18]),hypothetical_contact_time_seconds=.004,
        book_level_gravity_is_hypothesis=True,actual_camera_hand_registration_verified=False,physical_contact_parameters_identified=False,
        model_is_prescribed_measured_hand_path_plus_hypothetical_finish=True,
        new_root_motor_ids=[8,12],new_root_extra_ticks=128,new_root_travel_degrees=11.25,new_finish_seconds=3.2,
        native_empty_fixture=FIXTURE.relative_to(ROOT).as_posix(),native_empty_proof_sha256=sha(FIXTURE/'screen.json'),static_cases=len(proof['static_screens']),native_dynamic_cases=len(proof['dynamic_sensitivity']),native_screen_seconds=proof['elapsed_seconds'],
        device_free_tests_passed=7,device_free_test_source_sha256=sha(ROOT/'test_quarter_finish_empty_20261025.py'),
        empty_motion_goals=len(rows),nominal_empty_goal_time_seconds=len(rows)*.05,last_goal_ticks=rows[-1][1].tolist(),
        early_thumb_recovery='Up to 64 ticks of remaining ID2 opening begins in the existing initial ring/tip release. No new destination beyond the known start.',
        initial_contact_bounds_and_actual_health_stops_unchanged=True,failed_development_fixtures_preserved=['finish_empty_fixture_v1','finish_empty_fixture_v2'],
        initial_separation_failed_versions_not_reissued=True,assistant_readonly_source='finish_empty_readonly_20261025_230039_588520/result.json',assistant_motor_goal_writes=0,assistant_torque_changes=0,
        cable_under_cube_operator_reply='케이블은 옆에 있었음',cube_on_cable_at_captured_start_operator_report=False,
        physical_ninety_degree_rotation_verified=False,loaded_trial_ready=False,supervised_empty_finish_trial_ready=True,
        next_user_command='leap_quarter_finish_empty.cmd',next_action='Cube OUT of hand and camera for the entire run. One terminal SPACE starts fresh OFF screening and EMPTY motion. Automatic OFF after the new finish and hold. Send done for saved-file review.',
        preview_gif=(base/'preview/level_book_finish_candidate.gif').relative_to(ROOT).as_posix(),preview_start_old_new=(base/'preview/start_old_end_new_end.png').relative_to(ROOT).as_posix(),protected_original_hashes=hashes,poster_sha256=sha(poster))
    write(reviewdir+'/root128_candidate_review.json',report)
    photo=read(reviewdir+'/review.json');photo.update(cable_contact_under_cube_unresolved=False,cube_on_cable_at_captured_start_operator_report=False,cable_operator_reply='케이블은 옆에 있었음',new_root128_candidate_review=report,next_action=report['next_action']);write(reviewdir+'/review.json',photo)
    targets=('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json')
    if set(targets)&set(extra_hashes()):raise ValueError('Status update would alter a bound source')
    for p in targets:
        r=read(p);r.update(timestamp=stamp,quarter_support_photo_completed=True,additional_quarter_support_photo_needed=False,
            root128_empty_finish_readiness=report,next_user_command=report['next_user_command'],next_user_command_ready=True,next_user_command_empty_only=True,
            next_user_command_photo_only=False,next_user_command_has_loaded_motion=False,rotation_in_next_command=False,
            supervised_experimental_loaded_trial_ready=False,robust_loaded_execution_ready=False,physical_ninety_degree_rotation_verified=False,
            unchanged_loaded_trial_repeat_requested=False,next_action=report['next_action']);write(p,r)
    text='''## CURRENT 2026-10-25: 새 마무리 후보 27개 조건 비교 완료 / 빈손 확인 명령 준비

- 측면 촬영은 완료됐으며 다시 요청하지 않는다. 사용자는 큐브 밑에 케이블이 깔린 것이 아니라 옆에 있었다고 답했다. 이 촬영의 배치에 대한 확인으로 기록한다.
- 책이 수평이라는 가정과 명시한 단단한 접촉 조건에서, 기존 실제 적재 관절 경로 뒤에 검지와 중지 뿌리 ID8 및 ID12의 목표를 각각 128틱 더 올리는 3.2초 마무리 후보를 계산했다. 손가락 추가 이동은 11.25도이며 큐브의 목표는 여전히 약 90도다. 마무리 뒤 큐브가 넘어진 후 손 안에 남는 조건을 함께 평가했다.
- 앞뒤 42/44/46mm 및 좌우 12/15/18mm의 모델 배치 9개와 큐브 마찰 가정 0.2/0.3/0.5를 합친 27개 조건이 방향과 유지 및 1mm 이하 모델 겹침 기준을 만족했다. 실제 배치를 카메라와 정합한 값은 아니며 실물 마찰과 접촉 강도, 적재 모터 응답은 아직 식별하지 않았다. 하나의 이어지는 구간에서는 계산 시간 간격 1ms와 0.5ms에서도 마무리 끝 방향이 비슷했다. 기본의 부드러운 접촉에서는 큰 겹침이 남으므로 조건 없이 성공한다고 표시하지 않는다.
- 실제 장치에서는 OFF 관절값 세 번만 읽었으며 최대 차이는 1틱이었다. 이 출발값의 엄지 중간 관절 ID2는 전원을 끈 뒤 더 접혀 있었다. 새 빈손 복귀는 남은 ID2 펴기 중 최대 64틱을 기존 초기 분리 구간에서 시작한다. 접촉 허용치와 기한을 늘리지 않았다. 초기 순서를 수정한 전체 손 경로는 정적 201개 및 모터 응답 9개 조건과 독립 검증을 통과했다. OFF 및 복원과 중간 중단, 적재 허용 위조 거부를 포함한 장치 없는 프로그램 검사 7개도 통과했다. 실패한 개발 검사 v1 및 v2는 보존했다.
- 다음 사용자 명령은 **`leap_quarter_finish_empty.cmd`**다. **큐브는 처음부터 끝까지 손과 카메라 밖에 둔다. PLACE 단계는 없다.** 손가락과 받침을 그대로 두고 터미널 SPACE 한 번으로 시작한다. 신선한 OFF 검사 후 알려진 경로와 새 마무리까지 약 51초 목표 구간을 수행하고 마지막 정지 뒤 자동 OFF한다. 검사 시간은 별도이며 Q와 Esc 및 Ctrl+C는 중단이다. 완료 뒤 했어만 알리면 저장 파일을 확인한다.
- 현재 명령은 빈손 확인이며 큐브를 올린 새 마무리 실험은 아직 요청하지 않는다. 실제 90도 회전은 미검증이다. 시뮬레이션 영상은 `right_hand_rotation_20261024/support_photo_review_221703/level_book_seating/preview/level_book_finish_candidate.gif`, 상세 근거는 `root128_candidate_review.json`에 있다. 실제 모터 목표와 설정 및 토크 쓰기는 이번 준비에서 0회이며 원본 시퀀스, 실제 기록과 전시 문구의 해시를 확인했다.

<!-- END ROOT128 EMPTY FINISH READY 20261025 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8-sig');marker='<!-- END ROOT128 EMPTY FINISH READY 20261025 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip('\n')
        old=old.replace('## CURRENT 2026-10-25: 측면 받침 촬영 완료','## PREVIOUS 2026-10-25: 측면 받침 촬영 완료',1);p.write_text(text+old,encoding='utf-8')
    validate_proof(plan,FIXTURE/'screen.json')
    print(json.dumps(dict(empty_finish_ready=True,comparison_cases=27,static_cases=201,native_cases=9,program_tests=7,command=report['next_user_command'],nominal_motion_seconds=report['nominal_empty_goal_time_seconds']),ensure_ascii=False))
if __name__=='__main__':main()
