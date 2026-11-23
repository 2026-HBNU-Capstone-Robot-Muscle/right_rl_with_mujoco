"""Record the latest requested grasp/cycle scope and bounded offline evidence."""
from pathlib import Path
from datetime import datetime,timezone,timedelta
import hashlib,json
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261116/opposed_lower_grip'


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    frozen_path=ROOT/'right_hand_rotation_20261106/middle_clearance_probe/review_centered_v2.json'
    frozen=json.loads(frozen_path.read_bytes())['code_hashes']
    for name,digest in frozen.items():
        if sha(ROOT/name)!=digest:raise RuntimeError('Previously reviewed source changed: '+name)
    hold=json.loads((OUT/'review.json').read_bytes())['hold']
    static_reports=[OUT/'three_finger_trials/review.json',OUT/'three_finger_trials_span12_back20_back_only/review.json']
    cases=[case for path in static_reports for case in json.loads(path.read_bytes())['cases']]
    intent=dict(
        timestamp=datetime.now(timezone(timedelta(hours=9))).isoformat(),
        requested_front_contact='thumb on lower front face',
        requested_back_contacts=['index','middle'],back_rule='index OR middle OR both; choose by measured reach and stability',
        ring_required=False,under_support_counts_as_support=True,upper_layer_must_remain_clear=True,
        cycle=['establish_common_lower_grip','whole_cube_rotation_1','return_to_common_grip_and_hold',
               'whole_cube_rotation_2','return_to_common_grip_and_hold','repeat_on_request'],
        final_grip_end_state_must_seed_next_rotation=True,
        no_cube_or_hand_state_reset_between_motion_segments=True,
        front_back_model_axes_are_not_registered_to_current_camera_colors=True,
        camera_detection_is_separate_incomplete_work=True,
        old_TH_MF_RF_pose_is_reference_not_required_new_target=True,
        independent_hold_completed=hold['completed_goals']==hold['total_goals'],
        independent_hold_seconds=hold['full_fixed_hold']['seconds'],
        initial_settling_max_shift_mm=hold['full_fixed_hold']['center_shift_mm'],
        final_tilt_deg=hold['final_up_tilt_deg'],
        upper_sweep_minimum_sampled_gap_mm=hold['settled_tail']['minimum_upper_gap_sampled_75ms_mm'],
        posture_accepted=False,three_finger_static_cases=len(cases),
        three_finger_static_passes=sum(c['geometry_passed'] for c in cases),
        reaching_path_verified=False,continuous_cycle_verified=False,
        physical_grasp_verified=False,hardware_execution_allowed=False,
        devices_opened=False,frozen_source_count=len(frozen),frozen_sources_unchanged=True)
    (OUT/'cycle_spec.json').write_text(json.dumps(intent,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    files=['inspect_opposed_grip_20261116.py','prepare_opposed_lower_grip_20261116.py',
        'inspect_opposed_hold_clearance_20261116.py','prepare_opposed_three_finger_20261116.py',
        'review_opposed_contact_candidates_20261116.py','finish_opposed_lower_grip_20261116.py']
    manifest=dict(sources={name:sha(ROOT/name) for name in files},
        evidence={p.relative_to(ROOT).as_posix():sha(p) for p in [OUT/'review.json',OUT/'actual_contact_review.json',*static_reports]},
        hardware_execution_allowed=False,frozen_sources_unchanged=True)
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    (OUT/'README.md').write_text(
        '# 앞뒤 하부 파지와 반복 회전 — 2026-11-16\n\n'
        '사용자의 최신 목표는 엄지가 앞면 아래쪽을 막고 검지·중지 중 하나 또는 둘이 반대쪽을 지지하는 파지다. '
        '손 위 받침도 무게 지지로 인정한다. 약지를 필수 접촉으로 고정하지 않는다. '
        '앞·뒤는 현재 사진의 색과 등록된 좌표가 아니며 이번 모델 비교에서는 엄지 쪽 큐브 +Z/반대쪽 -Z로 정의했다.\n\n'
        '순서는 공통 하부 파지 준비 → 큐브 전체 회전 → 공통 파지 복귀·유지 → 다음 회전 → 같은 파지·유지다. '
        '각 구간의 끝 상태를 다음 구간에 그대로 넘겨야 하며, 별도로 초기화한 파지나 회전의 성공을 연속 실행 성공으로 세지 않는다.\n\n'
        '## 이번에 확인한 내용\n\n'
        '- 기존 엄지·검지 앞뒤 후보를 복사하지 않고 출처 해시와 모델 접촉을 확인했다.\n'
        '- 후보 자세를 독립적으로 초기화한 자유 큐브 유지 시험에서 134목표/10.05초를 완료했다. '
        '이것은 현재 실물 자세에서 도달했다는 근거가 아니다.\n'
        '- 모델의 기존 접촉·관절·추종 경계는 통과했고 원본 170개 결합 해시는 유지됐다. '
        '실물·카메라 접근 및 모터 지령은 없었다.\n'
        '- 중지를 뒷면 또는 아래 받침으로 더하는 첫 4후보와, 배치·뒷손가락 간격을 바꾼 2후보를 검사했다. '
        '기하학적 참고점과 실제 MuJoCo 접촉을 별도로 비교했다.\n\n'
        '## 남은 문제\n\n'
        f'- 두 손가락 후보는 유지 중 처음부터 최대 {hold["full_fixed_hold"]["center_shift_mm"]:.2f}mm 이동해 기울기 {hold["final_up_tilt_deg"]:.2f}도로 가라앉았다. '
        f'윗단 회전 영역과의 최소 여유는 {hold["settled_tail"]["minimum_upper_gap_sampled_75ms_mm"]:.2f}mm라서 공통 파지 후보로 채택하지 않았다. '
        '끝의 안정된 4.05초만 보고 초기 이동을 생략하지 않는다.\n'
        '- 중지 추가 6후보에는 실제 큐브-중지 접촉이 없었다. 모델 참고점이 면 가장자리 밖에 있다는 이유만으로 단정한 결과가 아니다.\n'
        '- 공통 파지 자세 보완, 회전 뒤 그 자세로 들어가는 전환, 그 끝 상태에서 다음 회전 및 반복 검증이 남았다.\n'
        '- 새 실물 실행파일은 준비하지 않았다. 모델 0.15N·m는 실제 450mA 환산값이 아니다. '
        '현재 임시 배치의 실물 좌표 보정도 미확인이다.\n',encoding='utf-8')
    handoff=ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md'
    rel=handoff.relative_to(ROOT).as_posix()
    if rel in frozen:raise RuntimeError('Do not edit frozen handoff')
    marker='<!-- opposed_lower_grip_cycle_user_intent_20261116 -->'
    previous=handoff.read_text(encoding='utf-8-sig')
    if marker not in previous:
        note=(marker+'\n## 최신 우선순위: 앞뒤 하부 파지 → 회전·파지 유지 반복\n\n'
            '사용자가 엄지로 큐브 앞면 아래를 막고 검지·중지 중 하나 또는 둘로 뒤를 지지하는 파지를 제안했다. '
            '이 파지를 먼저 만들고 회전→공통 파지 유지→다음 회전→공통 파지 유지로 연결한다. 약지 접촉을 필수로 고정하지 않는다. '
            '마커 없는 카메라 입력은 별도의 미완료 작업이며 파지 연구를 모두 끝났다고 가정하지 않는다.\n\n'
            '새 오프라인 확인: 기존 엄지·검지 후보는 독립 초기화 후 10.05초 유지했지만 초기 최대11.03mm 이동·최종19.57도 기울기와 '
            '윗단 여유 -5.87mm로 공통 파지 채택은 실패다. 중지 추가6후보는 실제 중지 접촉이 없다. '
            '새 파지/회전 연결이나 실물 시험 준비가 완료됐다고 안내하지 않는다. 실물 장치 접근0, 원본170해시 보존. '
            '출처와 다음 단계: `right_hand_rotation_20261116/opposed_lower_grip/README.md`, `cycle_spec.json`.\n'
            '<!-- /opposed_lower_grip_cycle_user_intent_20261116 -->\n\n')
        handoff.write_text(note+previous,encoding='utf-8')
    print(json.dumps(dict(common_grip_accepted=False,hold_seconds=10.05,static_cases=len(cases),
        frozen_sources_unchanged=True,report=str(OUT/'README.md')),ensure_ascii=False))


if __name__=='__main__':main()
