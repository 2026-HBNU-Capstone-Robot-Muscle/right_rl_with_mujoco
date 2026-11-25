"""Record requested lower bound and a completed command with no verified flip."""
import datetime,hashlib,json
from pathlib import Path
from loaded_quarter_trial_plan_20261025 import LoadedQuarterTrialPlan,ROOT
from loaded_quarter_trial_gate_20261025 import validate_proof
from loaded_quarter_joint_limits_20261025 import setting_metadata
OUT=ROOT/'right_hand_rotation_20261024/actual_loaded_review_171607'
F=ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/fixture_v6'
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,r):Path(p).write_text(json.dumps(r,indent=2,ensure_ascii=False),encoding='utf-8')
def main():
    plan=LoadedQuarterTrialPlan(F/'request.json');proof=validate_proof(plan,F/'screen.json')
    log=(F/'test_completion.log').read_text(encoding='utf-8');assert 'Ran 22 tests' in log and log.rstrip().endswith('OK')
    actual=json.loads((OUT/'loaded_completion_review.json').read_bytes());timeline=json.loads((OUT/'loaded_video_timeline.json').read_bytes())
    first=json.loads((OUT/'actual_loaded_contact_hypotheses.json').read_bytes());friction=json.loads((OUT/'actual_loaded_finish_candidates_v2.json').read_bytes())
    finish=json.loads((OUT/'actual_loaded_stalled_finish_candidates_v3.json').read_bytes());search=json.loads((OUT/'finish_contact_search.json').read_bytes())
    baseline=json.loads((ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json').read_bytes())
    poster='exhibition_20261023/족자_붙여넣기.txt';assert digest(ROOT/poster)==baseline['production_hashes_recorded_without_reissuing_certificates'][poster]
    assert digest(ROOT/'right_hand_rotation_20261023/native_motor_limits.json')=='c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716'
    assert digest(Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'))=='008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'
    assert digest(ROOT/'sequence_flip_poses.json')=='cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b'
    assert actual['full_loaded_command_samples']==760 and actual['all_torque_off_verified'] and actual['settings_restored']
    assert digest(ROOT/actual['source']/'result.json')==actual['source_sha256']
    now=datetime.datetime.now().astimezone().isoformat()
    simple=lambda c:{k:c[k] for k in ('offset_xy_mm','hypothetical_finish_extra_root_ticks','final_rotation_from_settled_deg','final_target_error_deg','external_cube_contact','maximum_no_hand_contact_s')}
    report=dict(timestamp=now,user_requested_ID4_lower=setting_metadata(),fixture=F.relative_to(ROOT).as_posix(),
        static_route_cases=len(proof['static_screens']),native_route_cases=len(proof['dynamic_sensitivity']),device_free_tests_passed=22,
        scoped_route_verification_passed=True,actual_loaded_trial=actual,video_usable_grid_frames=timeline['usable'],video_review_frames=timeline['frames'],
        provisional_video_windows=timeline['windows'],physical_quarter_turn_verified=False,robust_loaded_execution_ready=False,
        initial_cube_pose_in_hand_not_registered=True,physical_contact_friction_not_identified=True,
        nominal_low_friction_position_cases=[simple(c) for c in first['cases'] if c['initial_orientation']=='prior_model'],
        cube_friction_default=friction['original_cube_friction'],cube_friction_high_hypothesis=friction['diagnostic_cube_friction'],
        higher_friction_position_cases=[simple(c) for c in friction['cases'] if c['hypothetical_finish_extra_root_ticks']==0],
        rejected_root_finish_cases=[simple(c) for c in finish['cases']],rejected_contact_finish_candidates=len(search['cases']),
        accepted_contact_finish_candidates=len(search['accepted']),
        high_friction_model_success_is_not_physical_success=True,large_rotation_magnitude_after_drop_is_not_success=True,
        next_user_command=None,next_user_action='Confirm whether existing cube-contact fingertip surfaces are rigid plastic or compliant material; no motor rerun.',
        unchanged_loaded_trial_repeat_requested=False,assistant_hardware_commands_sent=0,assistant_device_acquisition=False,
        poster_unchanged=True,original_reference_and_original_native_file_unchanged=True,
        official_fingertip_material_reference='https://v1.leaphand.com/leap_cad',
        official_material_note='Official v1 CAD recommends soft fingertips and lists optional Ninjaflex Edge prints; this does not identify actual friction.')
    save(OUT/'combined_review.json',report)
    readiness=ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'
    ready=json.loads(readiness.read_bytes());backup=readiness.with_name('readiness_before_actual_loaded_review.json')
    if not backup.exists():backup.write_bytes(readiness.read_bytes())
    ready.update(timestamp=now,supervised_experimental_loaded_trial_ready=False,robust_loaded_execution_ready=False,
        physical_cube_rotation_verified=False,physical_loaded_reference_was_executed=True,actual_loaded_trial=actual,
        fixture=report['fixture'],plan_version=plan.PLAN_VERSION,source_request_sha256=plan.SOURCE_SHA,
        code_hashes=proof['code_hashes'],extra_code_hashes=proof['extra_code_hashes'],device_free_tests_passed=22,
        scoped_ID4_lower_setting=setting_metadata(),assistant_motor_goal_writes=0,assistant_torque_changes=0,
        next_user_command=None,unchanged_loaded_trial_repeat_requested=False,current_review=(OUT/'combined_review.json').relative_to(ROOT).as_posix(),
        native_ranges_widened_in_limits_file=False,only_existing_observed_id4_passive_reentry_used=False,
        user_requested_scoped_ID4_lower_degrees=2,measured_id4_native_min_before_next_phase=1798)
    save(readiness,ready)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json'):
        p=ROOT/name;r=json.loads(p.read_bytes());r.update(timestamp=now,next_user_command=None,next_user_command_ready=False,
            rotation_in_next_command=False,supervised_experimental_loaded_trial_ready=False,reverse_loaded_execution_ready=False,
            robust_loaded_execution_ready=False,physical_ninety_degree_rotation_verified=False,loaded_reference_physically_executed=True,
            actual_loaded_trial=actual,loaded_trial_readiness=ready,scoped_ID4_lower_setting=setting_metadata(),
            new_device_free_tests_passed=22,new_agent_motor_goal_writes=0,new_agent_torque_changes=0,
            next_action=report['next_user_action'],unchanged_loaded_trial_repeat_requested=False,current_loaded_review=report)
        save(p,r)
    note='''## CURRENT 2026-10-25: 전체 적재 명령 완료 / 큐브 뒤집기 미달 검토 / ID4 하한 2도 반영

- 17:16 실행 `loaded_quarter_trial_20261025_171607_019752`에서 전체 적재 목표 760개와 정지 관찰을 마쳤으며, 큐브 제거 확인과 토크 OFF 및 설정 복원을 확인했다. 최대 추종 0.1043rad, 최대 전류 122mA, 최저 전압 3.8V, 최대 온도 24도다. 이는 명령 실행 완료이며 큐브 90도 뒤집기 성공과 다르다.
- 사용자는 홈에 걸쳐 약 45도 기울었고 넘겨지지 않은 것 같다고 답했으며 기억은 불확실하다고 했다. 45도를 계측값으로 기록하지 않는다. 전후 사진과 전체 동작의 저장 프레임 323장을 검토했다. 큰 초록 면의 격자 추정이 유효한 139장에서 후반에만 더 기울고 마지막에 유지되는 양상이 나왔다. 보정 카메라 일치와 본체 3D 이동이 재검증되지 않았으므로 이 각도 추정을 정밀 실물 측정으로 표시하지 않는다. 실제 90도 성공은 미검증이다.
- 요청한 엄지 뿌리 ID4 하한 2도 감소를 현재 감독 실험의 소프트웨어와 모델에 한정해 반영했다. 연속 하한은 1820.4874틱에서 1797.7318틱, 정수 최소 목표는 1821틱에서 1798틱이다. 나머지 관절 범위 및 상한은 그대로다. 원본 범위 JSON과 EEPROM을 변경하지 않았다. 수정한 손 경로는 정적 201개 조건과 동역학 9개 조건, 독립 검증 및 장치 없는 검사 22개를 통과했다. 이 검사는 큐브가 없는 손 경로 검사이며 실물 회전 검증은 아니다.
- 실제 적재 관절 궤적을 처방한 별도 접촉 진단에서는 마찰과 큐브 시작 위치에 따라 결과가 크게 달랐다. 낮은 큐브 마찰 가정 0.3에서 기준 위치는 약 100도 움직였지만 손목 쪽 4mm 조건은 약 34도, 반대쪽 4mm 조건은 약 41도에 머물렀다. 높은 마찰 가정 0.8에서는 세 조건이 넘어갔다. 이 값들은 실물 마찰 측정값이 아니며, 실제 원인을 위치나 마찰 하나로 확정하지 않는다. 기존 시뮬레이션의 성공을 이번 실물 성공으로 옮겨 적지 않는다.
- 단순히 두 손가락 뿌리를 더 올리는 후보는 작은 변화로는 충분히 넘어가지 않았고 큰 변화에서는 큐브를 놓치는 조건이 나왔다. 큰 회전량만 보고 성공으로 분류하지 않았다. 엄지 접촉을 추가한 27개 후보도 유지와 접촉 기준을 함께 통과하지 못해 실행 경로에 넣지 않았다. 기존 적재 경로를 그대로 반복하도록 요청하지 않는다.
- 다음 사용자 확인은 큐브와 닿는 기존 손끝이 단단한 플라스틱인지 탄성 재질인지다. 모터 재실행은 필요 없다. [공식 LEAP v1 CAD](https://v1.leaphand.com/leap_cad)는 용도에 맞춘 부드러운 손끝을 권장하고 선택형 Ninjaflex Edge 출력을 안내한다. 이는 현재 손의 재질이나 마찰을 확인한 근거는 아니다.
- 세부 근거와 비교 영상 표는 `right_hand_rotation_20261024/actual_loaded_review_171607/combined_review.json`과 `loaded_turn_contact_sheet.jpg`에 저장했다. 이번 분석에서 실제 장치를 열지 않았고 모터 쓰기는 0회다. 실제 기록, 원본 시퀀스와 전시 문구는 보존했다.

<!-- END ACTUAL PARTIAL AND ID4 LOWER 20261025 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8')
        if old.startswith('## CURRENT 2026-10-25: 전체 적재 명령 완료'):old=old.split('<!-- END ACTUAL PARTIAL AND ID4 LOWER 20261025 -->',1)[1].lstrip()
        p.write_text(note+old.replace('## CURRENT','## PREVIOUS'),encoding='utf-8')
    (OUT/'README.md').write_text(note,encoding='utf-8')
    print(json.dumps(dict(lower_setting=setting_metadata(),static=201,native=9,tests=22,quarter_turn_verified=False,
        hardware_commands_sent=0,next_user_command=None,poster_unchanged=True),ensure_ascii=False))
if __name__=='__main__':main()
