"""Persist EMPTY-only readiness and preserve loaded-model limitations."""
import datetime,hashlib,json
from pathlib import Path
from reverse_quarter_path_empty_plan_20261025 import ROOT,ReverseQuarterEmptyPlan,TURN_PHASE,motion_goals,MOTION_FILE
from reverse_quarter_path_empty_gate_20261025 import validate_proof
OUT=ROOT/'right_hand_rotation_20261024/reverse_quarter_path_empty'
FIXTURE=OUT/'fixture_v1'
MODEL=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_075405_loaded_model'

def main():
    p=ReverseQuarterEmptyPlan(FIXTURE/'request.json');r=validate_proof(p,FIXTURE/'screen.json')
    tests=(FIXTURE/'test_completion.log').read_text(encoding='utf-8');assert 'Ran 13 tests' in tests and tests.rstrip().endswith('OK')
    raw=motion_goals();rows=p.plan(p.source_position());nominal=(len(rows)+20)*.05
    model=json.loads((MODEL/'short_visualization_manifest.json').read_bytes())
    empty=json.loads((MODEL/'empty_tip24_ring56_check.json').read_bytes())
    assert len(empty['cases'])==3 and all(c['complete_model_pass'] for c in empty['cases'])
    original=json.loads((ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json').read_bytes())
    poster=ROOT/'exhibition_20261023/족자_붙여넣기.txt';assert hashlib.sha256(poster.read_bytes()).hexdigest()==original['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    ready=dict(timestamp=datetime.datetime.now().astimezone().isoformat(),empty_whole_path_ready=True,
        user_command='leap_reverse_quarter_path_empty.cmd',cube_allowed=False,cube_placement_checkpoint=False,
        whole_reference_motion_goals=len(raw)-1,whole_reference_motion_seconds=(len(raw)-1)*.05,total_recorded_goals=len(rows),
        nominal_total_motor_on_seconds=nominal,full_static_cases=len(r['static_screens']),full_native_dynamic_cases=len(r['dynamic_sensitivity']),
        device_free_tests_passed=13,source_request_sha256=p.SOURCE_SHA,plan_sha256=p.plan_digest(p.source_position()),
        goal_data_sha256=hashlib.sha256((ROOT/MOTION_FILE).read_bytes()).hexdigest(),code_hashes=r['code_hashes'],extra_code_hashes=r['extra_code_hashes'],
        actual_current_PID_speed_and_stops_unchanged=True,operator_terminal_space_count=1,torque_off_automatic=True,
        assistant_motor_goal_writes=0,assistant_torque_changes=0,loaded_cube_execution_ready=False,
        physical_cube_rotation_verified=False,nominal_model_rotation_deg=model['whole_cube_rotation_deg'],
        nominal_model_final_orientation_error_deg=model['final_target_error_deg'],
        model_limit='Nominal model only. Weak motor response and some placement/friction cases do not achieve the loaded orientation. The current command is EMPTY only.')
    (OUT/'readiness_review.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
    # Older search v2 evaluated the end of a batch-padded hold. Preserve its
    # original numbers and explicitly route fixed-duration claims to the replay.
    legacy=MODEL/'tip_clearance_search_v2/summary.json';old=json.loads(legacy.read_bytes())
    old.update(batch_padding_extends_short_candidate_hold=True,short_candidate_actual_search_hold_seconds=31.9,
        fixed_four_second_recheck='short_quarter_native_verification.json',not_used_for_current_empty_runtime=True)
    legacy.write_text(json.dumps(old,indent=2),encoding='utf-8')
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json'):
        path=ROOT/name;status=json.loads(path.read_bytes())
        status.update(timestamp=ready['timestamp'],next_user_command=ready['user_command'],next_user_command_ready=True,
            next_user_command_empty_only=True,next_user_command_photo_only=False,whole_quarter_empty_readiness=ready,
            next_action='User performs one EMPTY whole-reference path check with terminal SPACE once. Cube remains outside the hand and camera for the entire run. Review actual trajectory before any loaded rotation.',
            reverse_loaded_execution_ready=False,physical_ninety_degree_rotation_verified=False)
        path.write_text(json.dumps(status,ensure_ascii=False,indent=2),encoding='utf-8')
    text=f'''## CURRENT 2026-10-25: 큐브 배치 확인 완료 / 90도 후보 전체 경로 빈손 실행 준비

- 새 배치 촬영 `reverse_start_placement_20261025_075405_164917`은 정상 완료됐다. 실제 큐브가 검지와 중지 쪽 평평한 윗면에 받쳐진 사진과 REMOVE 확인 및 OFF와 설정 복원을 확인했다. 배치 촬영은 다시 요청하지 않는다. 사진의 사람 손은 본체에 보이며 큐브에 닿아 보이지 않는다. 정밀 3차원 정합이나 파지 힘을 검증한 것은 아니다.
- 새 기준 조건 후보는 엄지 끝 목표를 24틱, 약지 끝 목표를 56틱 펴고, 목표 방향에 도달한 뒤의 불필요한 관절 구간을 제거한 38초 경로다. 저장된 모델 상태의 시작과 끝에서 큐브 전체 변화 약 {model['whole_cube_rotation_deg']:.1f}도와 최종 목표 오차 {model['final_target_error_deg']:.1f}도가 보인다. 이는 기준 조건의 시뮬레이션이며 실제 성공이 아니다. 낮은 모터 응답과 일부 마찰 및 배치 조건에서 목표에 미달하므로 적재 실행 준비로 표시하지 않는다. 그림은 `reverse_empty_entry/actual_075405_loaded_model/short_quarter_start_and_finish.png`다.
- 다음 사용자 작업은 **`leap_reverse_quarter_path_empty.cmd`**다. 처음부터 끝까지 큐브를 손과 화면 밖에 둔다. 터미널 SPACE 한 번으로 새 현재 OFF 값을 읽고 검사한 뒤 복귀, 2초 시작 정지, 전체 38초 관절 경로와 최종 정지 후 자동 OFF한다. PLACE 단계나 사진용 추가 SPACE는 없다. 카메라는 자동으로 열린다. 현재 입력의 전체 모터 ON 명목 시간은 약 {nominal:.1f}초이며 OFF 모델 검사와 카메라 시작 시간은 별도다. 손가락과 받침은 그대로 두고 끝나면 했어만 알려주면 된다.
- `reverse_quarter_path_empty/fixture_v1`의 전체 201개 정적 조건과 9개 동역학 조건, 독립 Windows 검증과 장치 없는 검사 13개를 통과했다. 별도 빈손 구간도 세 모터 응답 조건에서 자기 겹침 없이 통과했다. 현재 100초 제한 안에 들어가며 실제 전류와 PID, 2틱/50ms 목표 제한, 전압과 추종 및 전류 중단 기준은 그대로다. 실행 때 새 OFF 읽기와 전체 경로 검사 없이는 움직이지 않는다. 실제 모터 쓰기는 이번 준비에서 0회다.
- 실물 전체 경로를 확인하기 전 큐브 적재 회전을 요청하지 않는다. 저장될 빈손 관절값은 실물 응답과 모델의 차이를 평가하는 입력이다. 기존 모델의 작은 패드 겹침을 허용치 확대로 통과시키지 않았다. 원본 실제 기록과 전시 문구는 변경하지 않았다.

<!-- END EMPTY QUARTER READY 20261025 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith('## CURRENT 2026-10-25: 큐브 배치 확인 완료'):
            old=old.split('<!-- END EMPTY QUARTER READY 20261025 -->',1)[1].lstrip()
        path.write_text(text+old.replace('## CURRENT','## PREVIOUS'),encoding='utf-8')
    (OUT/'README.md').write_text('''# 90도 후보의 전체 관절 경로 빈손 확인

현재 명령은 `leap_reverse_quarter_path_empty.cmd`다. 큐브는 처음부터 끝까지 손과 화면 밖에 둔다. 카메라는 자동으로 열리고 터미널 SPACE 한 번으로 현재 OFF 측정과 전체 검사를 거친다. 짧은 복귀와 시작 정지 뒤 전체 38초 경로를 한 번 움직이고 자동 OFF한다. 추가 SPACE나 PLACE 단계는 없다. 명목 전체 ON 시간은 약 48초이며 검사 시간은 별도다. Q 또는 Esc 및 Ctrl+C로 중단한다.

이 단계는 실물에서 후보 전체 경로의 추종과 간섭을 확인한다. 실제 큐브 회전이나 라이브 SAC가 아니다. 실행 결과의 카메라 영상과 실제 관절값을 직접 검토하기 전 적재 실행을 요청하지 않는다. 끝나면 했어만 알려주면 된다.

`fixture_v1`은 완료된 실제 촬영의 안정 OFF 값을 출발점으로 사용한 개발 입력이다. 201개 정적 시작/받침 조건과 9개 전체 동역학 조건, 독립 Windows 검증과 장치 없는 검사 13개를 통과했다. 실행 때는 그 개발 입력이 아니라 새 안정 OFF 값으로 다시 검사한다. `readiness_review.json`에 해시와 검사 범위를 기록했다.

제안된 목표는 `quarter_turn_goals_v1.json`에 실제 측정 궤적과 구분해 저장했다. 기존 원본과 실제 기록은 수정하지 않았다. 모델의 모터 응답과 실제 큐브 배치는 완전 보정되지 않았고 약한 응답의 적재 조건에서는 목표 미달이 있다. 기준 조건의 회전 그림을 물리 성공으로 해석하지 않는다. 전시 문구는 수정하지 않았다.
''',encoding='utf-8')
    print(json.dumps({k:ready[k] for k in ('empty_whole_path_ready','user_command','nominal_total_motor_on_seconds','device_free_tests_passed','loaded_cube_execution_ready')},ensure_ascii=False))

if __name__=='__main__':main()
