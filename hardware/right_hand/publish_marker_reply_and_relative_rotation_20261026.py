"""Refresh handoff after measurement reply; never request the same capture."""
import hashlib,json
from datetime import datetime
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026'
def main():
    relative=json.loads((OUT/'relative_rotation_review.json').read_bytes());dim=json.loads((OUT/'red_marker_range_sensitivity.json').read_bytes());qualified=[c for c in relative['cases'] if c.get('qualified')]
    state=dict(status='marker_reply_resolved_relative_rotation_analysis_and_ring_clearance_planning',updated_local=datetime.now().isoformat(timespec='seconds'),red_marker_user_range_mm=[10.5,10.7],red_marker_nominal_sensitivity_mm=10.6,red_marker_measurement_precise=False,photo_9_8mm_adopted_as_physical_size=False,between_size_position_shift_mm=dim['between_size_median_shift_norm_mm'],relative_rotation_physical_size_required=False,relative_rotation_synthetic_checks=18,saved_source_relative_yaw_deg=qualified[-1]['best']['reference_axis_twist_deg'] if qualified else None,saved_source_two_marker_disagreement_deg=qualified[-1]['best']['between_cube_marker_rotation_disagreement_deg'] if qualified else None,physical_vertical_successes_total=2,physical_vertical_successes_consecutive=False,physical_horizontal_90_verified=False,offline_two_tip_grasp_found=True,offline_90_yaw_candidate_verified=False,passive_ring_clearance_study_in_progress=True,next_user_action=None,next_user_command=None,additional_capture_required=False,additional_marker_measurement_required=False,ready_for_real_yaw=False,hardware_execution_allowed=False,real_devices_opened_this_turn=False,motor_commands_sent_this_turn=0)
    proof=json.loads((OUT/'legacy_stage4_ring_uncurl_other_v1/empty_path_response_review.json').read_bytes())
    state.update(status='awaiting_5V_power_for_fresh_readonly_start_angles',passive_ring_clearance_study_in_progress=False,passive_ring_clearance_endpoint_variants_tested=4,passive_ring_free_cube_regrasp_variants_tested=2,lift_squeeze_diagnostics_completed=3,lift_squeeze_90_candidate_accepted=False,empty_grasp_transition_geometry_passed=proof['all_passed'],empty_grasp_transition_samples=proof['samples'],empty_response_fractions_tested=[0.,.25,.5,.75,1.],fresh_off_recovery_path_screened=False,usb_com7_present=True,serial_port_opened_this_turn=False,next_user_action='큐브를 손 밖에 둔 채 5V 모터 전원을 켜고 알려주기. 손가락과 받침 유지. 프로그램 실행 불필요.',next_user_command=None,pending_user_input='5V 모터 전원 ON 및 큐브 제거 확인',next_assistant_action_after_reply='hand_readonly_check.inspect_bus의 PING/READ만 사용해 현재 각도 확인. 목표/토크/설정 쓰기 금지.',additional_capture_required=False)
    for rel in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        p=ROOT/rel;old=json.loads(p.read_bytes()) if p.exists() else {};old.update(state);p.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (OUT/'marker_reply_and_relative_rotation_handoff.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block='''## CURRENT 2026-10-26: 빨강 치수 답변 반영, 회전량과 위치 추정 분리

- 빨강 ID2는 사용자 답변 그대로 약 10.5~10.7mm로 기록했다. 정확한 실측으로 격상하지 않으며 계산용 10.6mm와 양 끝값을 비교했다. 9.8mm 사진 추정치를 실제 치수로 채택하지 않았다. 같은 크기 질문과 추가 촬영은 현재 필요 없다.
- 저장 영상에서 같은 마커의 전후 회전을 비교하는 별도 분석을 만들었다. 마커 크기와 면 중심 오프셋은 이 상대 회전에서 소거된다. 두 큐브 마커와 손 기준 마커를 함께 비교하고 평면 자세 후보의 모호함을 남긴다. 카메라가 고정된 경우와 움직인 경우의 0도 및 ±90도 합성 사례를 세 크기로 계산한 18개 검사가 통과했다. 실측 각도 정밀도 보증이나 라이브 카메라/SAC 제어 완료는 아니다.
- 10월 11일 05:15의 stage_4_end에서 stage_6_eligible까지 저장된 두 큐브 마커는 약 0도 변화로 분석됐다(개별 마커 회전 추정 불일치 약 3.39도). 이 구간이 실물 수평 90도 성공이라는 근거는 없다. 같은 자료로 만든 자유 큐브 모델은 약 -25도를 예측해 차이가 있으므로 과거 움직임과 일치하는 확정 모델로 표현하지 않는다.
- 두 면이 보이는 과거 네 번째 자세에서 엄지/중지 두 손끝 파지가 모델상 마지막 고정 구간까지 유지되는 후보를 찾았다. 회전을 이어 붙이면 한 방향 약 32도에서 중지/약지의 모델 간섭이 나타나며 아직 90도 후보는 승인하지 않았다. 약지를 먼저 펴는 네 변형의 끝 자세를 비교하고 두 후보의 자유 큐브 파지를 재생했다. 통로가 생기는 후보도 있지만 회전 중 파지가 풀리거나 기울었고, 세 가지 압착/들기 진단도 90도 성공으로 승인하지 않았다. 충돌 허용치와 모터 전류/PID/전압/추종 기준은 넓히지 않았다.
- 약지를 펴고 엄지/중지를 접근시키는 **빈손 내부 경로** 606개 자세는 명령 각도와 과거 실측 오차의 0/25/50/75/100% 다섯 경우에서 손 자체 및 받침 겹침 없이 통과했다. 전체 오차 포락선 검증이나 현재 손에서 출발하는 복귀 경로 검증은 아니다. `legacy_stage4_ring_uncurl_other_v1/empty_grasp_goal_views.png`는 큐브를 숨긴 모델 제안 그림이며 실물 재현 사진이 아니다.
- 다음 사용자 작업이 생겼다. USB 포트 목록만 읽어 COM7이 있음을 확인했다(시리얼 포트 개방/모터 패킷 0). 촬영 때 꺼둔 5V 모터 전원을 켜고 큐브가 손 밖에 있음을 알려주도록 요청했다. 손가락/받침을 맞추거나 프로그램을 실행할 필요는 없다. 답변 후 현재 각도를 PING/READ로만 확인해 새 빈손 복귀 경로의 출발점으로 삼는다. 그 전에는 모터 포트를 열지 않는다.
- 실물 수직 뒤집기 성공 총 2회는 보존한다. 수평 90도 실물 성공과 새 실물 실행 준비 완료는 아니다. 이 작업은 저장 파일과 오프라인 모델만 사용했고 모터·카메라를 열지 않았다. 새 목표 전송/토크/설정 쓰기는 승인하거나 수행하지 않았다.

<!-- END MARKER REPLY RELATIVE ROTATION 20261026 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8-sig');marker='<!-- END MARKER REPLY RELATIVE ROTATION 20261026 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip()
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        p.write_text(block+old,encoding='utf-8')
    protected={'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b','right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716','exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0'}
    for path,expected in protected.items():assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==expected,path
    print(json.dumps(state,ensure_ascii=False,indent=2));print('Protected source/native ranges/poster hashes unchanged.')
if __name__=='__main__':main()
