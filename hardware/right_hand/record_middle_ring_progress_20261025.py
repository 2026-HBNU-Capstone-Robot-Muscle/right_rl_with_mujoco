"""Record completed side-view review and offline planning, preserving raw data."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
SIDE=ROOT/'right_hand_rotation_20261024/middle_ring_side_review'
PLAN=ROOT/'right_hand_rotation_20261024/original_sequence_clearance'


def main():
    pairs=json.loads((SIDE/'paired_capture_summary.json').read_bytes())
    geometry=json.loads((SIDE/'paired_native_geometry.json').read_bytes())
    original=json.loads((ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json').read_bytes())
    poster=ROOT/'exhibition_20261023/족자_붙여넣기.txt'
    assert hashlib.sha256(poster.read_bytes()).hexdigest()==original['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    assert pairs['same_encoder_pose_within_3_ticks']
    for row in pairs['captures']:
        blob=(ROOT/row['folder']/'result.json').read_bytes()
        assert hashlib.sha256(blob).hexdigest()==row['result_sha256']
    gaps=[r['closest_middle_ring_hull_pairs'][0]['model_gap_mm'] for r in geometry['captures']]
    assert min(gaps)>0 and all(not any(c['kind']=='middle_ring_mesh_hulls' for c in r['contacts']) for r in geometry['captures'])
    status=dict(side_photo_input_complete=True,additional_side_photo_needed=False,
        source_captures=[r['folder'] for r in pairs['captures']],
        maximum_between_capture_delta_ticks=pairs['maximum_between_capture_delta_ticks'],
        source_motor_goal_writes=0,source_torque_changes=0,agent_new_motor_writes=0,
        middle_ring_hull_model_gap_mm=gaps,physical_gap_measured=False,
        geometric_calibration_verified=False,original_recorded_sequence_used_as_reference=True,
        original_sequence_first_transition_adaptations_rejected=True,
        separate_teacher_horizontal_segments_static_clear=True,
        nominal_free_cube_horizontal_yaw_about_45_degrees=True,
        robust_physical_ninety_degree_rotation_verified=False,
        poster_unchanged=True,new_hardware_command_ready=False,
        next_agent_work='Free-cube regrasp route validation, then finish remaining yaw; preserve actual motor safeguards.')
    dynamic=PLAN/'free_cube_regrasp_review_v3.json'
    if dynamic.exists():
        data=json.loads(dynamic.read_bytes())
        status['free_cube_regrasp_candidate_count']=len(data['simulated_regrasp_candidates'])
        status['free_cube_regrasp_review_file']=dynamic.relative_to(ROOT).as_posix()
    (SIDE/'progress_20261025.json').write_text(json.dumps(status,indent=2,ensure_ascii=False),encoding='utf-8')
    text='''# 2026-10-25 옆면 짝 사진 검토와 원본 기반 회전 계획

23:05:12와 23:05:55의 두 촬영을 확인했다. 두 기록 모두 사진 전후의 관절 읽기에 성공했고 토크와 오류 및 watchdog은 OFF와 정상 상태다. 촬영 내 위치 차이는 각각 최대 1틱과 0틱, 두 촬영의 대표 위치 차이는 최대 0.5틱이다. 두 실행 모두 20장 묶음을 저장했다. 이 입력을 위해 다시 촬영할 필요는 없다.

두 사진에 당시 관절값으로 재구성한 손 외형을 겹쳤다. 마커 기준 변환은 예전 수동 치수를 사용하는 임시 값이다. 마커 자체의 작은 픽셀 잔차가 손 전체의 보정 정밀도를 뜻하지 않는다. 중지와 약지 방향은 정성적으로 비교할 수 있으나 실제 힘이나 동작 중 접촉을 측정한 것은 아니다.

현재 정지 자세에서 추가한 중지와 약지 메시의 볼록 외형은 겹치지 않는다. 모델의 최근접 간격은 약 30.36mm지만 실제 간격을 잰 수치는 아니다. 이전 파지 및 회전 구간의 접촉 여부를 이 정지 사진만으로 확정하지 않는다. 실제 입력의 엄지 뿌리 ID4는 원래 모델 하한보다 약 6.46도 더 펴진 수동 OFF 자세다. 모델 영점이나 실물 목표 범위를 수정하지 않았다.

원본 7개 자세를 새 경로의 기준으로 사용했다. 처음 수직 동작 구간의 간섭 회피 시도는 중간 경로 검사에서 실패해 제외했다. 별도로 원본 자세 2에서 4까지의 밀기 구간과 자세 6까지의 연결 구간은 각 중간 목표에 대해 기존 충돌 도형과 추가 중지 및 약지 외형을 검사했다. 이것은 손 궤적의 정적 검사이며 큐브 회전 성공이나 모터 응답 검증이 아니다.

자유 큐브의 한 배치 및 마찰 가정에서는 자세 2에서 4까지의 느린 밀기로 수평 약 45도와 3초 유지가 나왔다. 뒤 동작을 연결하면 이탈하는 경우가 있었고, 추가 접촉 탐색도 90도와 유지 조건을 만족하지 못했다. 시뮬레이션 결과를 실물 성공으로 기록하지 않는다.

45도 뒤 큐브를 다시 잡기 위한 끝 자세를 역기구학으로 탐색했다. 큐브를 고정한 역기구학 계산은 목표 자세 생성에만 사용하며 성공 영상으로 사용하지 않는다. 자유 큐브 검증에서는 큐브를 고정하거나 목표 방향으로 강제로 옮기지 않는다. 곧장 재파지하는 경로에는 중지와 약지의 교차가 있어 제외했고, 약지를 먼저 빼는 순서를 계산한다.

새 실물 명령은 아직 준비 완료 상태가 아니다. 현재 사용자에게 추가 사진이나 같은 회전 재실행을 요청하지 않는다. 실제 전류, PID, 전압과 추종 중단 조건, 원본 파일과 전시 문구를 유지했다.

파일: `two_saved_side_views.jpg`, `side_2_overlay_branch_0.jpg`, `paired_pose_model_views.jpg`, `paired_capture_summary.json`, `paired_native_geometry.json`. 원본 기반 경로 및 자유 큐브 결과는 이 폴더 옆의 `original_sequence_clearance`에 저장한다.
'''
    (SIDE/'README.md').write_text(text,encoding='utf-8')
    prefix='''## CURRENT 2026-10-25: 옆면 짝 촬영 2회 확인 완료 / 원본 엄지 밀기와 재파지로 전체 90도 계획 진행

- 사용자가 같은 자세로 두 번 찍었다고 알린 `pose_pair_readonly_20261024_230512_916136`와 `230555_418562`를 확인했다. 모두 정상 저장, 촬영 내 최대 드리프트 1틱과 0틱, 촬영 간 대표 위치 차이 최대 0.5틱이다. 모터 쓰기와 토크 변경은 0회다. 이전의 옆면 사진 요청은 충족됐으며 반복 요청하지 않는다.
- 두 사진에 현재 모터 대응과 실제 관절값으로 외형을 겹쳤다. 현재 정지 자세의 중지와 약지 추가 외형은 모델에서 분리돼 있고 방향을 정성 비교할 수 있다. 완전한 3차원 보정이나 동작 중 접촉 검증은 아니다. 엄지 ID4의 수동 OFF 하한 초과를 보정값으로 숨기지 않았다.
- 원본 7개 자세 중 첫 수직 구간의 새 경로는 중간 간섭 검사에서 실패해 제외했다. 별도의 원본 자세 2에서 4까지 밀기 구간은 정적 검사 후 자유 큐브에서 일부 조건의 수평 약 45도와 유지가 나왔다. 자세 6까지 그대로 잇거나 제한된 추가 접촉 동작으로 90도 유지에 도달하지 못했다. 이 결과는 시뮬레이션 가정에 해당하며 실물 성공으로 표시하지 않는다.
- 후속 계산은 45도 이후 재파지와 남은 회전을 연결하는 전체 경로다. 중지와 약지의 교차 경로는 제외하고 약지 공간 확보 후 재파지 순서를 평가한다. 역기구학용 고정 큐브 계산과 자유 큐브 동역학 평가를 구분한다. 현재 새 실물 실행 명령은 준비되지 않았다. 사진이나 작은 회전 시험을 다시 요청하지 않는다.
- 근거: `right_hand_rotation_20261024/middle_ring_side_review/README.md`, `progress_20261025.json`, `original_sequence_clearance/`. 실제 전류와 PID 및 중단 기준, 원본과 전시 문구는 변경하지 않았다.

'''
    p=ROOT/'sac_current_status.md';old=p.read_text(encoding='utf-8')
    if not old.startswith(prefix):p.write_text(prefix+old,encoding='utf-8')
    official='''# 공식 자료 재확인

[LEAP Hand IsaacGym 저장소](https://github.com/leap-hand/LEAP_Hand_Sim/blob/master/README.md)는 사전 학습 정책과 안정 파지 캐시 생성 절차를 제공한다. 기본 실물 설명은 7.5cm 큐브 조건이다.

[LEAP Hand Isaac Lab 저장소](https://github.com/leap-hand/LEAP_Hand_Isaac_Lab)는 정책 실물 실행과 7.5cm PLA 큐브 281g의 실험 조건을 설명한다. 현재 사용자의 55mm, 100g 큐브 및 받침 조건이 그대로 보정됐다는 근거로 사용하지 않는다.

공식 정책은 추가 검토 기준이다. 현재 손의 관절 대응, 실제 모터 응답과 초기 파지를 확인하지 않은 채 실물에 실행하거나 공식 예제의 설정으로 현재 설정을 덮어쓰지 않았다.
'''
    (SIDE/'official_reference_notes.md').write_text(official,encoding='utf-8')
    print(json.dumps(status,indent=2))


if __name__=='__main__':main()
