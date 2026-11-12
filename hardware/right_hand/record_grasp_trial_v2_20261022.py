"""Record the 02:30 no-write stop and bounded-current-recovery correction."""
import hashlib
import json
from pathlib import Path
from grasp_place_plan import ROOT,GraspPlacePlan,completed_source,MAX_EMPTY_RECOVERY_TICKS
from thumb_live_start_gate import validate_proof

def main():
    out=ROOT/'prepared_cube_review_20261022'
    folder=ROOT/'cube_grasp_trial_20261022_023055_407697'
    data=(folder/'result.json').read_bytes();r=json.loads(data)
    assert r['status']=='stopped' and not r['write_events'] and r['all_torque_off_verified']
    fixture=out/'grasp_trial_v2_fixture';plan=GraspPlacePlan(fixture/'request.json')
    proof=validate_proof(plan,fixture/'screen.json')
    position=r['preflight_state']['position'];previous=completed_source()['final_off_snapshots'][-1]['position']
    diagnosis=dict(source=str((folder/'result.json').relative_to(ROOT)),source_sha256=hashlib.sha256(data).hexdigest(),
        original_status=r['status'],failed_phase=r['failed_phase'],write_attempts=0,
        all_torque_off_verified=True,device_error_flags=r['preflight_state']['error'],
        minimum_voltage_v=min(r['preflight_state']['voltage'])/10,
        maximum_voltage_v=max(r['preflight_state']['voltage'])/10,
        previous_off_ticks=previous,current_off_ticks=position,
        id15_previous_off_ticks=previous[14],id15_current_off_ticks=position[14],
        id15_change_ticks=position[14]-previous[14],
        id15_return_to_completed_goal_ticks=position[14]-completed_source()['final_on_state']['goal'][14],
        interpretation='Historical pose-lock rejected later OFF movement before writes. Actual cause of passive movement is not proven by these snapshots.',
        settings_restored_false_because_no_setting_writes=True)
    (folder/'stop_review.json').write_text(json.dumps(diagnosis,ensure_ascii=False,indent=2),encoding='utf-8')
    task='큐브를 빼고 손은 그대로. 같은 leap_cube_grasp_trial.cmd 실행, v2 배너 확인. 첫 SPACE는 빈손 복귀, PLACE 후 큐브 배치/손 제거/두 번째 SPACE는 접근, REMOVE 후 큐브 제거/세 번째 SPACE는 OFF. 결과 로그 전달.'
    ready=dict(status='bounded_current_recovery_cube_trial_v2_ready',plan_version=plan.PLAN_VERSION,
        launcher='leap_cube_grasp_trial.cmd',source_stop=diagnosis,
        historical_pose_lock_removed=True,max_empty_recovery_ticks=MAX_EMPTY_RECOVERY_TICKS,
        max_empty_recovery_degrees=MAX_EMPTY_RECOVERY_TICKS*360/4096,
        unchanged_cube_approach_goals=True,fresh_stable_off_and_path_screen_required=True,
        static_cases=len(proof['static_screens']),native_ranges_unchanged=proof['native_ranges_unchanged'],
        maximum_unapproved_range_violation_rad=max(s['unapproved_native_range_violation_rad'] for s in proof['static_screens']),
        maximum_self_overlap_mm=max(s['max_self_overlap_mm'] for s in proof['static_screens']),
        maximum_unapproved_self_overlap_mm=max(s['self_escape_max_unapproved_overlap_mm'] for s in proof['static_screens']),
        modeled_initial_self_contact_is_reported_not_zeroed=True,
        static_contact_separated_by_goal=max(s['self_escape_complete_at'] for s in proof['static_screens']),
        dynamic=proof['dynamic_sensitivity'],fixture_motion_samples=len(plan.plan(plan.source_position())),
        source_is_not_new_stable_hardware_read=True,tests_passed=94,user_task=task,
        proof='prepared_cube_review_20261022/grasp_trial_v2_fixture/windows_integration.json',
        hardware_commands_sent_by_assistant=0,physical_grasp_verified=False,physical_rotation_verified=False)
    (out/'grasp_trial_v2_ready.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
    p=out/'current_status.json';state=json.loads(p.read_text(encoding='utf-8'))
    state.update(status=ready['status'],next_user_task=task,next_motion_launcher=ready['launcher'],
        latest_user_stop=diagnosis,next_trial_evidence='prepared_cube_review_20261022/grasp_trial_v2_ready.json')
    p.write_text(json.dumps(state,ensure_ascii=False,indent=2),encoding='utf-8')
    p=ROOT/'right_hand_start_20261019/objective_and_schedule.json';state=json.loads(p.read_text(encoding='utf-8'))
    state.update(current_user_task=task,
        current_blocker='Latest cube trial stopped before writes at historical OFF pose gate; v2 screens bounded recovery from current angles. Loaded approach and cube rotations remain unverified.',
        latest_user_hardware_trial=diagnosis,latest_offline_review=ready['proof'],
        next_trial_review='prepared_cube_review_20261022/grasp_trial_v2_ready.json')
    p.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    note='''## CURRENT 2026-10-22 02:30 큐브 시험 preflight 중단 / 현재 각도 복귀 v2

- `cube_grasp_trial_20261022_023055_407697`: 설정/목표 쓰기0회, 전체 토크 OFF 확인, 오류 비트0, 전압4.7~5.0V. 중단은 ID15=2847이 과거 ON/OFF 각도 기준2561..2724 밖이라 생김. settings_restored=False는 복원할 쓰기가 없었던 실행이며 복원 실패가 아니다. 원본 stopped 결과 보존.
- ID15는 직전 짧은 OFF 표본2660에서2847로187틱(16.44°) 변했고, 완료 목표2625까지 복귀량은222틱(19.51°). 사진도 빈손이며 다른 관절은 지난 OFF 값에서8틱 이내 변화. 단일 OFF 순간을 이후 자세로 고정한 소프트웨어 시작 조건이 부적절했다. 중력/케이블/사람 접촉 중 무엇이 원인인지 이 기록만으로 확정하지 않는다.
- `CUBE GRASP TRIAL v2`는 과거 OFF 각도와의 일치 조건을 제거. 현재 원시 각도를 읽어 완료 목표까지 각 관절 최대512틱(45°)인 빈손 복귀만 검사 대상으로 접수한다. 3개 안정 OFF 읽기, 실제 시작±4틱67개 정적 경로, 원래 관절·제어 범위와 기존 단조 복귀 제한, gain0.5/1/1.5 동역학을 통과해야만 첫 쓰기 가능. 이 변경은 임의 자세의 충돌 회피 완료를 의미하지 않는다.
- 이번 저장 자세의 모델은 약지 끝/손바닥4·6·7 세 충돌 도형에 최초 약0.98/1.28/1.13mm 겹침을 검출했다. 기존의 자기 접촉0 조건 때문에 첫 재검사가 거부되었으며 그 실패 자료도 남김. 복귀 목표는 이 세 깊이를 각각 단조 감소시키고20개 목표 이전에 분리함. 실물 관통 깊이를 측정한 값은 아니다.
- 새 검사에서는 첫 빈손 회복에서만 지정된 초기 약지 끝/손바닥 접촉(초기 모델 깊이2mm 이하)을 분리할 수 있다. 각 명령에서 접촉 깊이 단조 감소, 새 접촉/더 깊어짐 금지,40개 목표(2초) 내 분리를 요구. 동역학의 초기 얕은 반동은 초기 깊이를 넘을 수 없으며 초기 유지 포함2.5초 내 완전히 분리해야 하고 이후 접촉 금지. 이를 일반적인 자기 접촉 허용으로 바꾸지 않는다. 모델 범위나 접촉 도형/마찰/질량 변경 없음.
- 이번 저장 자세에서775개 목표(명목38.75초) Windows→WSL 실제 검사 통과.67개 사례 초기 모델 겹침 최대1.633mm를 그대로 기록하며 허용되지 않은 겹침0, 받침 겹침0, 허용되지 않은 범위 위반0. 동역학 세 강도에서 접촉 분리 약1.60/1.51/1.47초, 최대 추종0.05561/0.02795/0.02633rad. 전압/전류/PID/추종/읽기 지연/55초 이동 제한 유지. 큐브가 있는322개 접근 목표와 PLACE/REMOVE 확인 흐름 변경 없음. 실제 실행은 다시 현재 각도로 검사한다.
- 새 실제 ID15=2847 회귀 사례와16관절 이동량 상한/전압/회전수 거부, 기존 접촉 단조 분리·새 접촉·깊어짐·지연·NaN 거부를 포함 **94개 테스트 통과**. 조수 카메라·모터 접근 없음. 다음은 같은 명령을 v2로 한 번 실행; 손을 과거 모양으로 수동 복원하거나 같은 촬영을 따로 할 필요 없음. 큐브 배치/파지/회전 실물 성공은 아직 미검증.
- 사용자 작업: 큐브를 빼고 같은 `leap_cube_grasp_trial.cmd`, v2 배너 확인 → 첫 터미널 SPACE. PLACE 후 초록 위·주황 카메라 쪽 배치/손 제거/두 번째 SPACE. REMOVE 후 큐브 제거/세 번째 SPACE로 OFF. 로그 전달.
- 근거: 중단 폴더 `stop_review.json`, `prepared_cube_review_20261022/grasp_trial_v2_fixture/windows_integration.json`, `grasp_trial_v2_ready.json`.

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        p=ROOT/name;content=p.read_text(encoding='utf-8')
        if note.splitlines()[0] not in content:p.write_text(note+content,encoding='utf-8')
    print(json.dumps({k:ready[k] for k in ('status','tests_passed','fixture_motion_samples','dynamic')},indent=2))

if __name__=='__main__':main()
