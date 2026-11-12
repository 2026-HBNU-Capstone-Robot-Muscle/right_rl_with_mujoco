"""Pin the successful paired capture and tested next operator action."""
import hashlib
import json
from pathlib import Path
import numpy as np

from hand_motor_mapping import TICK
from slow_grasp_demo_plan import SlowGraspDemoPlan,endpoints
from thumb_live_start_gate import validate_proof

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_pose_readonly_20261023_171233_601943'


def main():
    blob=(FOLDER/'result.json').read_bytes();source=json.loads(blob)
    assert source['status']=='readonly_cube_pose_pairs_saved' and source['usable_pair_count']==10
    assert source['all_completed_pairs_have_off_endpoints'] and source['camera_restoration_verified']
    assert max(row['maximum_drift_ticks'] for row in source['pairs'])==1
    poses=json.loads((FOLDER/'direct_marker_pose_review.json').read_bytes())
    assert poses['source_sha256']==hashlib.sha256(blob).hexdigest()
    validation_path=ROOT/'right_hand_rotation_20261023/slow_grasp_demo_validation.json'
    validation=json.loads(validation_path.read_bytes())
    assert validation['passed'] and validation['tests_run']==210
    assert validation['existing_native_fixtures_passed']==validation['existing_fixtures_replayed']==21
    fixture=ROOT/'right_hand_rotation_20261023/slow_grasp_demo_fixture'
    plan=SlowGraspDemoPlan(fixture/'fresh_off_request.json')
    proof=validate_proof(plan,fixture/'native_screen.json')
    opened,closed=endpoints();raw=plan.source_position()
    grid=next(r for r in poses['branch_summaries']
        if r['source'].endswith('grid_diagnostic') and r['palm_branch']==0)
    record=dict(status='paired_capture_reviewed_slow_grasp_demo_ready',
        source=FOLDER.name,source_sha256=hashlib.sha256(blob).hexdigest(),paired_encoder_images=12,
        directly_decoded_palm_cube_frames=10,max_encoder_endpoint_drift_ticks=1,all_sampled_torque_off=True,
        camera_original_settings_restored=True,
        observed_closed_goal_index_pip_difference_ticks=int(closed[6]-raw[6]),
        observed_closed_goal_index_pip_difference_degrees=float((closed[6]-raw[6])*TICK*180/np.pi),
        grid_diagnostic_coordinate_spread_mm=grid['coordinate_spread_mm'],
        grid_is_not_general_scrambled_cube_tracker=True,marker_to_model_registration_verified=False,
        pose_analysis_used_to_change_loaded_cube_target=False,
        next_launcher='leap_slow_grasp_demo.cmd',new_plan_version=plan.PLAN_VERSION,
        targets_count=len(plan.plan(raw)),nominal_motion_seconds=len(plan.plan(raw))*.05,
        motion_deadline_seconds=plan.ON_TIMEOUT,observation_hold_seconds=3.,
        initial_separation_command_limit=plan.INITIAL_SELF_ESCAPE_GOALS,
        initial_separation_pip_travel_limit_ticks=plan.MAX_CONTACT_RELEASE_TRAVEL_TICKS,
        initial_dynamic_separation_limit_seconds=plan.INITIAL_SELF_ESCAPE_DYNAMIC_S,
        maximum_static_actual_separation_command=max(r['self_escape_complete_at'] for r in proof['static_screens']),
        maximum_static_actual_separation_pip_ticks=max(r['contact_clear_pip_travel_ticks'] for r in proof['static_screens']),
        original_grasp_limits_changed=False,new_or_deeper_contact_permitted=False,
        drive_current_pid_tracking_voltage_limits_changed=False,
        old_historical_pose_lock_replaced_by_per_joint_endpoint_distance=True,
        no_historical_file_reads_during_torque_on=True,
        camera_exposure_minus7_is_opt_in_for_demo=True,marker_loss_is_motor_gate=False,
        tests_passed=validation['tests_run'],static_start_cases=len(proof['static_screens']),
        dynamic_response_cases=len(proof['dynamic_sensitivity']),existing_native_fixtures_replayed=21,
        regression_report_sha256=hashlib.sha256(validation_path.read_bytes()).hexdigest(),
        proof_sha256=hashlib.sha256((fixture/'native_screen.json').read_bytes()).hexdigest(),
        physical_new_demo_executed=False,physical_cube_rotation_verified=False,live_sac=False,
        real_camera_or_serial_opened_by_assistant=False,hardware_commands_sent_by_assistant=0)
    (ROOT/'right_hand_rotation_20261023/slow_grasp_demo_ready_review.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
    title='## CURRENT 2026-10-23 17:12 이후: 동시 자료 10쌍 확보 / 느린 파지 시연 준비'
    note='''

- `cube_pose_readonly_20261023_171233_601943`에서 사진과 전후 관절 각도 12쌍을 저장했고 그중 10쌍에서 ID40과 ID5를 직접 함께 해독했다. 16개 토크는 모두 OFF였으며 각도 전후 차이는 최대 1틱이다. 카메라 원래 설정 복원도 확인했다. 이 읽기 전용 촬영을 다시 요청하지 않는다.
- 같은 사진의 두 평면 자세 분기를 모두 보존해 40개 마커 위치 후보를 계산했다. 초록 면의 넓은 8칸 격자를 독립 진단으로 추가하니 선택한 손바닥 분기의 좌표 반복 범위가 약 0.73, 1.37, 1.20mm였다. 작은 마커만 쓴 경우보다 반복성은 나아졌으나 절대 보정 정확도가 아니며, 색이 섞인 큐브용 추적기로 사용하지 않는다. 기존 마커 장착 좌표를 유지한 60개 모델 후보를 비교했지만 실물 접촉이나 새 회전 목표를 확정하지 않았다.
- 중요한 차이: 이번 자료는 OFF 자세다. 이전 파지 목표와 검지 PIP ID7이 521틱, 약 45.8도 다르다. OFF 사진의 손끝 간격을 동작 중 파지 간격으로 간주하지 않는다. 다음에는 이미 도달한 준비 및 파지 끝점을 이용한 느린 시연에서 ON 상태의 영상과 각도를 함께 기록한다.
- 새 `leap_slow_grasp_demo.cmd`: **큐브 제거 후 시작 → 약지 분리 확인 SPACE → 준비 자세 → PLACE 때 큐브 배치 및 SPACE → 느린 파지 → 3초 같은 목표 유지 → 큐브 제거 및 SPACE → OFF와 설정 복원**. 회전이나 SAC 실행은 포함되지 않는다. 노출 −7과 직접 해독 마커 표시를 새 카메라 형식에만 적용하며, 움직이는 동안 마커가 가려지는 것으로 중단하지 않는다. GUI는 별도 프로세스이며 원본 영상에 표시는 저장하지 않는다.
- 시작 검사는 관절별로 도달한 열린 끝점과 닫힌 끝점 중 가까운 값과의 거리를 사용한다. 서로 다른 관절이 다른 쪽으로 처진 혼합 OFF 자세를 한 과거 자세 전체에 강제로 맞추지 않는다. 각 관절 512틱 가족 및 원래 범위로 돌아오는 0.06rad 제한을 유지하고, 실제 시작 주변 ±4틱의 67정적 및 3응답 검사를 반드시 다시 수행한다. ON 이후 경로 계산은 메모리만 사용한다.
- 이 시연의 초기 약지 분리는 현재 주변 조건에서 최대 43명령 및 PIP 86틱이 필요했다. 새 시연만 44명령과 88틱 및 응답 안정화 3초로 제한한다. 접촉은 계속 줄어야 하고 새 접촉이나 더 깊은 접촉은 허용하지 않는다. 약지 끝의 기존 64틱 펴기를 앞당기고 전체 빈손 분리 구간 및 사용자 확인 전에 분리를 마친다. 기존 실행본의 40명령, 80틱, 2.5초는 그대로다. 전류 450mA, PID 및 전압, 전류, 추종 중단 기준도 그대로다.
- 저장된 시작에서 새 경로는 926명령, 명목 46.3초다. 지연된 60ms 주기의 예산까지 계산해 새 시연의 동작 제한을 60초로 정했다. 동일 목표 3초 유지도 경로 증명에 포함한다. 210개 장치 없는 회귀 검사 통과. 새 시작 67정적 및 3응답 통과, 기존 21개 시작도 현재 코드로 모두 Windows에서 WSL을 호출해 재검증했다. 과거 실물 실행의 원본 증명 해시는 보존했고 새 해시로 바꾸지 않았다.
- **지금 필요한 사용자 작업:** 큐브를 먼저 빼고 손가락 모양을 그대로 둔다. USB와 모터 전원을 연결하고 다른 프로그램을 닫은 뒤 `leap_slow_grasp_demo.cmd` 실행. 화면에서 ID40이 읽히도록 반사를 줄이고 첫 SPACE. 약지 분리 확인 SPACE. PLACE 때 초록 위와 주황 카메라 쪽으로 큐브를 놓고 손을 뺀 뒤 SPACE. 마지막 REMOVE 때 큐브를 빼고 SPACE. 결과와 영상은 자동 저장하므로 했어만 알려주면 된다.
- 다음 사용자 실행 결과는 `leap_slow_grasp_demo_result.json` 및 `slow_grasp_demo_날짜_시간/`에서 읽는다. 동작 중 파지 영상의 직접 해독 좌표와 전후 상태 표본을 맞춰 다음 회전 모델 및 경로를 비교한다. 전시용 완성 시점 문구는 실제 완료 증거로 사용하지 않는다.
- 근거: `right_hand_rotation_20261023/slow_grasp_demo_ready_review.json`, `slow_grasp_demo_validation.json`, `slow_grasp_demo_fixture/native_screen.json`, `leap_slow_grasp_demo_README.md`, 이번 촬영 폴더의 `direct_marker_pose_review.json`과 `native_geometry_review.json`.

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith(title):continue
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(title+note+old,encoding='utf-8')
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
