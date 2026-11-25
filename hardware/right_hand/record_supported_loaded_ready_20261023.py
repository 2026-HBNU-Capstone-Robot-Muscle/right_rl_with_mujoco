"""Record the completed empty observation and reviewable next loaded trial."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261023'

def main():
    from supported_cube_rotation_plan import SupportedCubeRotationPlan,candidate_record
    from thumb_live_start_gate import validate_proof
    empty=json.loads((ROOT/'supported_yaw_empty_20261023_030552_675209/completion_review.json').read_bytes())
    native=json.loads((ROOT/'right_hand_rotation_20261022/integration_validation.json').read_bytes())
    independent=json.loads((OUT/'supported_loaded_prefix_256_scale_085.json').read_bytes())
    tests=json.loads((OUT/'regression_tests.json').read_bytes())
    assert empty['exact_previous_targets'] and empty['final_target_exact']
    assert empty['all_torque_off_verified'] and empty['settings_restored']
    assert native['passed'] and len(native['fixtures'])==20 and tests['passed']
    assert independent['clearance_tracking_screen_passed'] and independent['total_motion_cases']==154
    assert independent['total_control_cases']==154
    candidate_record()
    for suffix in ('latest_off','prior'):
        folder=OUT/('fixture_loaded_supported_'+suffix)
        validate_proof(SupportedCubeRotationPlan(folder/'request.json'),folder/'screen.json')
    record=dict(status='bounded_loaded_observation_ready_for_operator',
        completed_empty_run='supported_yaw_empty_20261023_030552_675209',
        empty_suffix_goals_completed=256,empty_min_voltage_v=empty['min_voltage_v'],
        empty_peak_current_ma=empty['peak_absolute_current_ma'],empty_max_tracking_rad=empty['max_tracking_rad'],
        empty_torque_off_verified=True,empty_settings_restored=True,
        empty_camera_good_frames=empty['camera']['image_signal_usable_frames'],
        empty_camera_blank_frames=empty['camera']['image_signal_blank_frames'],
        last_off_endpoints_stable=empty['final_off_endpoint_stable'],
        stored_off_pose_is_not_automatic_start_acceptance=True,
        next_launcher='leap_cube_supported_rotation.cmd',plan_version=SupportedCubeRotationPlan.PLAN_VERSION,
        new_suffix_scale=.85,new_suffix_goals=256,new_suffix_nominal_seconds=12.8,
        independent_motion_cases_passed=154,independent_control_cases=154,
        requested_15_degree_rotation_passed=independent['requested_rotation_passed'],
        requested_15_degree_successful_cases=independent['successful_cases'],
        physical_rotation_verified=False,live_sac=False,algorithm='offline_CEM_fixed_path_NOT_SAC',
        requested_direction_guaranteed=False,real_cube_registration_verified=False,
        windows_wsl_starts_validated=20,tests_passed=tests['tests_run'],
        maximum_goal_step_ticks=2,goal_period_s=.05,current_cap_ma=450,motion_deadline_s=75,
        start_reentry_id4_max_rad=.08,other_joints_start_reentry_max_rad=.06,native_ranges_unchanged=True,
        agent_motor_or_camera_starts=0,
        next_action='Run EMPTY. SPACE starts; SPACE after ring tip/palm gap confirmation. PLACE CUBE prompt: green-center face up/orange-center toward camera; hands clear then SPACE. Observe small reorientation, REMOVE cube, SPACE for OFF. Q stops.')
    (OUT/'supported_loaded_ready_review.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    section=f'''## CURRENT 2026-10-23 03:05 이후: 빈손 새 경로 완주 / 큐브 포함 관찰 시험 준비

- 사용자 실행 `supported_yaw_empty_20261023_030552_675209` 완주. 초기 약지 분리 확인 이후 새 목표256개가 정확히 실행됐고 마지막 목표도 일치한다. 최저{empty['min_voltage_v']}V, 최대{empty['peak_absolute_current_ma']:.0f}mA, 최대 추종{empty['max_tracking_rad']:.5f}rad.16개OFF와 설정 복원 확인.970프레임 모두 정상이며 새 경로/유지111개 원본 프레임은 `recorded_empty_suffix.mp4`로 내보냈다. 큐브가 없었으므로 실물 큐브 회전 성공은 아니다.
- OFF 직후 엄지ID4와 약지가 중력으로 계속 처졌다. 마지막 두 OFF 표본은 안정하지 않았다. 기록된 마지막OFF값을 현재 시작 승인으로 사용하지 않고 매 실행의 새 안정OFF 읽기3개와67개 주변 경로/3개 응답 검사를 유지한다. ID4의 관측된 모형 범위 초과0.064rad와±4틱 주변(<0.071rad)을 반영해 새 큐브 시험의 초기 단조 복귀만0.08rad로 제한한다. 다른 관절은0.06rad 유지, 원래 관절/제어 범위를 변경하지 않는다. 기존 빈손 계획의0.06rad도 유지한다.
- 원래 크기256목표를 큐브 포함154조건과154무동작 대조군에서 재검사하니26조건이 추종0.12rad 기준을 초과했다. 전류/중단 기준을 바꾸지 않고 이동량85%로 줄인 경로는154조건 모두 충돌/낙하/추종 조건을 통과했다. 원래 합성 손 시작과 이번 실물 빈손 각도로 시작한 경우를 모두 포함한다. 목표15도 수평 회전은{independent['successful_cases']}/154개만 충족하므로 목표각/방향 보장 또는SAC 성공으로 기록하지 않는다. 큐브 초기 위치/모터 동역학은 아직 근사다.
- 새 `leap_cube_supported_rotation.cmd`는빈손분리/확인 → 열기 → PLACE/SPACE → 기존 느린쥐기 →85% 새 경로256개(명목12.8초) → REMOVE/SPACE → OFF다.450mA/기존PID,2틱/50ms,75초, 전압/온도/추종/진행/카메라/통신 중단, 무재시도/OFF 복원을 유지한다. 카메라는 함께 기록하며ID40게이트는 없다. 앞면/모서리 변화를 관찰한다. 수평 회전에서는 초록 윗면이 그대로일 수 있다.
- 새 실행본2개 시작을 포함한20개 Windows→WSL 저장 경로 검증과 회귀{tests['tests_run']}개 통과. 조수가 실제 장치를 열지 않았다.
- 사용자 할 일: 큐브를 빼고 현재 손을 그대로 두어 새cmd 실행. 첫SPACE, 약지 끝/손바닥 분리확인SPACE. PLACE 때 초록 중심면위·주황 중심면카메라 쪽으로 큐브를 놓고 손을 뺀 뒤SPACE. 끝나면 큐브 제거 후SPACE로OFF. 닿거나 걸리면Q. 했어만 알려주면 저장된 회전 구간과 각도를 직접 검토한다.
- 근거: `right_hand_rotation_20261023/supported_loaded_ready_review.json`, `supported_loaded_prefix_256_scale_085.json`, `regression_tests.json`, `right_hand_rotation_20261022/integration_validation.json`, 완료 빈손 폴더의 `completion_review.json`.

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if not old.startswith(section):path.write_text(section+old,encoding='utf-8')
    print(json.dumps({k:record[k] for k in ('status','next_launcher','tests_passed','windows_wsl_starts_validated','physical_rotation_verified')},indent=2))

if __name__=='__main__':main()
