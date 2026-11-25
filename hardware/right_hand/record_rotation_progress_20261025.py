"""Persist corrected evidence, current readiness and the next physical task."""
import datetime,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
SIDE=ROOT/'right_hand_rotation_20261024/middle_ring_side_review'
PLAN=ROOT/'right_hand_rotation_20261024/original_sequence_clearance'
ENTRY=ROOT/'right_hand_rotation_20261024/reverse_empty_entry'

def main():
    from reverse_start_empty_plan_20261025 import ReverseStartPlan
    from reverse_start_empty_gate_20261025 import validate_proof
    fixture=ENTRY/'fixture_v2';p=ReverseStartPlan(fixture/'request.json');proof=validate_proof(p,fixture/'screen.json')
    original=json.loads((ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json').read_bytes())
    poster=ROOT/'exhibition_20261023/족자_붙여넣기.txt'
    assert hashlib.sha256(poster.read_bytes()).hexdigest()==original['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    source=Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py')
    assert hashlib.sha256(source.read_bytes()).hexdigest()=='008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'
    pair=json.loads((SIDE/'paired_capture_summary.json').read_bytes());geometry=json.loads((SIDE/'paired_native_geometry.json').read_bytes())
    for capture in pair['captures']:
        assert hashlib.sha256((ROOT/capture['folder']/'result.json').read_bytes()).hexdigest()==capture['result_sha256']
    gaps=[r['closest_middle_ring_hull_pairs'][0]['model_gap_mm'] for r in geometry['captures']]
    verification=json.loads((PLAN/'reverse_original_native_verification.json').read_bytes())
    motion=[r for r in verification['runs'] if r.get('valid_start') and r.get('mode')=='moving_path']
    controls=[r for r in verification['runs'] if r.get('mode')=='matched_stationary_control']
    nominal=next(r for r in motion if r['gain_scale']==1 and r['friction']==.8 and r['offset_from_nominal_mm']==[0,0])
    assert nominal['simulated_orientation_and_retention_pass']
    assert not any(r['simulated_orientation_and_retention_pass'] for r in controls)
    actual_path='reverse_start_readonly_20261025_040743_585738/result.json';actual=json.loads((ROOT/actual_path).read_bytes())
    status=dict(timestamp=datetime.datetime.now().astimezone().isoformat(),
        side_photo_input_complete=True,additional_side_photo_needed=False,
        middle_ring_hull_model_gap_mm=gaps,physical_gap_measured=False,
        compiled_middle_ring_geometry='explicit mesh, validated against original visual transforms',
        old_30_36mm_sphere_proxy_gap_superseded=True,
        forward_reference_and_projected_SAC_failures_preserved=True,
        reverse_nominal_whole_cube_rotation_deg=nominal['whole_rotation_angle_deg'],
        reverse_nominal_goal_orientation_error_deg=nominal['final_target_error_deg'],
        reverse_nominal_red_up_error_deg=nominal['red_up_error_deg'],
        reverse_nominal_green_front_error_deg=nominal['green_front_error_deg'],
        reverse_nominal_native_tracking_rad=nominal['maximum_tracking_rad'],
        reverse_nominal_native_self_model_overlap_mm=nominal['maximum_self_model_overlap_mm'],
        reverse_valid_start_cases=len(motion),reverse_orientation_retention_pass_cases=sum(r['simulated_orientation_and_retention_pass'] for r in motion),
        reverse_stationary_control_cases=len(controls),reverse_stationary_control_goal_passes=0,
        reverse_loaded_execution_ready=False,physical_ninety_degree_rotation_verified=False,
        native_motor_parameters_identified=False,geometry_calibration_verified=False,
        source_readonly=actual_path,source_readonly_maximum_drift_ticks=actual['maximum_drift_ticks'],
        new_agent_motor_goal_writes=0,new_agent_torque_changes=0,
        next_user_command='leap_reverse_start_empty.cmd',next_user_command_empty_only=True,
        next_user_command_ready=True,empty_entry_goal_count=len(p.plan(p.source_position())),
        empty_entry_static_cases=len(proof['static_screens']),empty_entry_native_cases=len(proof['dynamic_sensitivity']),
        new_device_free_tests_passed=8,poster_unchanged=True,original_reference_unchanged=True,
        next_action='User runs EMPTY-only new start entry; agent then reviews saved camera and encoders before any loaded turn.')
    (SIDE/'progress_20261025.json').write_text(json.dumps(status,ensure_ascii=False,indent=2),encoding='utf-8')
    text=f'''# 2026-10-25 현재 회전 작업

옆면 짝 촬영 두 번은 모두 유효하다. 촬영 사이 대표 위치 차이는 최대 0.5틱이며 다시 촬영할 필요는 없다. 추가 중지 및 약지 외형 복사본이 일부 구형 도형으로 컴파일된 오류를 바로잡고, 10개 메시가 원래 CAD의 형상과 변환을 보존하는지 검증했다. 이전 약 30.36mm 간격 수치는 제외한다. 수정된 모델에서 최근접 간격은 약 {min(gaps):.2f}mm이며 실물 간격을 잰 값은 아니다. `proxy_analysis_superseded_20261025.json`을 함께 확인한다.

원본 `sequence_flip.py`의 7개 자세와 원본 해시를 보존했다. 새 손끝 CAD 모델은 기존 접촉 상자와 다른 둥근 형상을 비교하기 위한 오프라인 모델이다. 원래 물리 질량과 관절 범위 및 영점을 바꾸지 않았다. 이 외형이 실물에 완전히 보정됐다는 주장은 하지 않는다.

앞 순서와 수정한 SAC 경로는 자유 큐브 동역학에서 목표 회전에 실패했다. 정적 경로 통과와 큐브 회전 성공을 구분한다. 실패 결과는 삭제하지 않았다. 원본 앞부분은 모델에서 요청한 색 방향과 반대 방향으로 뒤집히는 경우가 있어, 자세 2에서 자세 0으로 되돌아오는 별도 후보를 평가했다.

역순 후보의 기준 모터 응답과 마찰 0.8 가정에서는 큐브 전체 변화 {nominal['whole_rotation_angle_deg']:.2f}도, 목표 자세 오차 {nominal['final_target_error_deg']:.2f}도로 4초 유지했다. 빨강 윗면 오차는 {nominal['red_up_error_deg']:.2f}도, 초록 앞면 오차는 {nominal['green_front_error_deg']:.2f}도다. 비교 시뮬레이션에서 초기 손 자세를 유지한 9개 조건은 목표에 도달하지 않았다. 기준 모터 응답 가정의 마찰 3종과 위치 변화 5종은 방향 및 유지 조건을 만족했다. 전체 45개 조합 중 7개는 초기 배치가 성립하지 않았고 나머지 38개 중 35개가 방향 및 유지 조건을 만족했다. 모터 응답이 약한 일부 조건은 실패했다. 이를 실물 성공률로 해석하지 않는다.

역순 후보의 native 동역학에는 패드 사이 최대 약 0.35mm의 모델 접촉이 남아 있어 적재 회전 실행 준비로 표시하지 않았다. 실제 침투 깊이나 힘을 측정한 값은 아니다. 간격 추가 경로의 목표 여유를 충족하지 못한 결과도 별도로 보존했다. 기존 충돌 기준을 완화해 적재 실행을 승인하지 않았다.

현재 사용자 작업은 `leap_reverse_start_empty.cmd`의 빈손 시작 자세 이동이다. USB 재연결 뒤 READ만 수행해 16개 OFF와 오류 없음, 최대 1틱 드리프트를 확인했다. 이 실제 값을 입력으로 한 전 구간 CAD 검사가 201개 정적 조건과 9개 동역학 조건을 통과했고 장치 없는 검사 8개도 통과했다. 프로그램은 실행 시 현재 OFF 관절값을 다시 읽고 별도 전체 검사를 수행한다. 한 단계 2틱과 50ms 주기, 기존 450mA 및 PID와 실제 추종 및 전압 중단 조건을 유지한다. 새 시작 자세까지 1303개 목표이며 약 65초의 명목 이동과 최종 정지 뒤 OFF다. 준비 자세 이동의 독립된 100초 제한은 이 경로의 지연 주기 예산으로 정했고 다른 프로그램 제한은 바꾸지 않았다.

큐브는 실행 내내 손과 화면 밖에 둔다. 터미널 SPACE 한 번으로 시작하며 PLACE 단계가 없다. 손가락과 책 및 본체는 그대로 둔다. 카메라 창은 자동으로 열린다. 이 실행은 큐브 회전이나 라이브 SAC가 아니다. 완료 뒤에는 저장된 사진과 실제 관절값을 직접 검토한다.

향후 큐브의 색 방향은 손을 기준으로 한다. 초록은 위, 주황은 손끝 반대인 손목 쪽이다. 카메라를 옮겨도 기준은 바꾸지 않는다. 역순 회전의 목표는 빨강 위와 초록 손목 쪽이다. 실제 적재 실행은 아직 요청하지 않는다.

시각화: `../original_sequence_clearance/reverse_turn_start_and_finish.png`, `reverse_turn_native_comparison.gif`. 물리 계산의 상태를 그대로 그린 그림이며 4배속이다. 단순 목표 표시용 가상 큐브는 렌더링에서 숨겼다. 렌더링 면과 카메라 방향도 수치로 확인했다. 실물 영상이 아니다.

실물 90도 성공은 아직 미확인이다. 이번 분석의 모터 목표 및 토크 쓰기는 0회다. 전시 문구는 변경하지 않았다.
'''
    (SIDE/'README.md').write_text(text,encoding='utf-8')
    (ENTRY/'README.md').write_text('''# 빈손 새 회전 시작 자세

`leap_reverse_start_empty.cmd` 실행. 큐브는 끝까지 손과 카메라 화면 밖에 둔다. 손가락과 받침은 그대로 둔다. 카메라는 자동으로 열리고 터미널 SPACE 한 번으로 현재 OFF 읽기와 검사 후 이동한다. 새 시작 자세만 잡고 OFF하며 큐브를 놓는 단계는 없다. 동작은 명목 약 65초이고, 그 전에 현재 자세의 전체 경로 검사 시간이 별도로 든다. Q 또는 Esc 및 Ctrl+C로 중단한다.

`fixture_v2/request.json`은 실제 READ 기록에 근거한 개발 검증 입력이다. `fixture_v2/screen.json`은 201개 정적 조건과 9개 동역학 조건을 모두 통과했다. 실행 때는 새 현재 OFF 값으로 다시 검증한다. 설정 복원과 OFF 확인 및 기존 실제 전압, 온도, 전류와 추종 중단을 유지한다.

큐브 회전 또는 라이브 SAC를 실행하는 명령이 아니다. 실물 입력을 검토한 뒤 다음 적재 단계를 정한다.
''',encoding='utf-8')
    marker='## CURRENT 2026-10-25: 역순 90도 조건부 시뮬레이션 / 빈손 새 시작 자세 실행 준비\n'
    prefix=marker+f'''
- 원본 기준 역순 후보는 모터 응답을 넣은 기준 조건에서 큐브 전체 약 95.6도와 목표 자세 오차 7.5도 및 4초 유지를 보였다. 빨강 위, 초록 손목 쪽이다. 정지 비교 9조건은 목표 회전에 실패했다. 이는 새 배치와 마찰 및 미보정 모터 가정의 시뮬레이션이며 실물 성공이 아니다. 패드 사이 모델 접촉이 남아 적재 회전 실행 준비는 아니다.
- 현재 READ `reverse_start_readonly_20261025_040743_585738`은 모두 OFF, 오류 없음, 최대 1틱 드리프트다. 새 준비 경로는 201개 정적 조건과 9개 native 동역학 조건 및 장치 없는 검사 8개를 통과했다. 다음 사용자 작업은 `leap_reverse_start_empty.cmd` 실행이다. 큐브는 끝까지 밖, 터미널 SPACE 한 번, 새 시작 자세만 이동 후 OFF다. 카메라는 자동으로 연다. 실제 동작이나 토크를 이번 도구 작업에서 쓰지 않았다.
- 기존 전류 및 PID와 실제 추종 및 전압 중단을 유지했다. 새 독립된 빈손 경로는 1303목표와 100초 제한이다. 실행 때 새 현재 OFF 값을 읽어 전체 검사를 다시 한다. 완료 뒤 저장된 영상과 관절값을 직접 검토한다.
- 이전 일부 외형 복사본의 구형 도형 오류를 수정했다. 옆면 정지 자세의 모델 간격은 약 10.28mm이며 이전 30.36mm는 제외한다. 당시 촬영 두 번은 충족된 입력이다. 재촬영이나 3도 진단 반복은 요청하지 않는다.
- 근거: `right_hand_rotation_20261024/middle_ring_side_review/README.md`, `original_sequence_clearance/reverse_original_native_verification.json`, `reverse_empty_entry/fixture_v2/`. 원본 및 실제 기록과 전시 문구는 보존했다.

<!-- END CURRENT ROTATION 20261025 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith(marker):old=old.split('<!-- END CURRENT ROTATION 20261025 -->',1)[1].lstrip()
        old=old.replace('## CURRENT','## PREVIOUS')
        path.write_text(prefix+old,encoding='utf-8')
    readiness=dict(empty_entry_ready=True,cube_allowed=False,rotation_included=False,source_request_sha256=p.SOURCE_SHA,
        plan_sha256=p.plan_digest(p.source_position()),static_cases=201,native_dynamic_cases=9,device_free_checks_passed=8,
        actual_motor_current_and_PID_unchanged=True,actual_motor_goal_and_torque_writes_sent=0,
        physical_ninety_degree_rotation_verified=False,user_command='leap_reverse_start_empty.cmd')
    (ENTRY/'readiness_review.json').write_text(json.dumps(readiness,indent=2),encoding='utf-8')
    (ROOT/'right_hand_rotation_20261024/current_handoff_20261025.json').write_text(json.dumps(status,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(readiness,indent=2))

if __name__=='__main__':main()
