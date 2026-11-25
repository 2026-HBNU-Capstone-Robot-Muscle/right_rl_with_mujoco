"""Mark the placement request fulfilled; never alter original trial records."""
import datetime,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_075405_placement_review'
review=json.loads((OUT/'review.json').read_bytes())
assert review['status']=='reverse_start_placement_complete' and review['cube_removal_confirmed'] and review['all_torque_off_verified'] and review['settings_restored']
source=ROOT/review['source'];assert hashlib.sha256(source.read_bytes()).hexdigest()==review['source_sha256']
details=dict(placement_capture_complete=True,source=review['source'],source_sha256=review['source_sha256'],
    max_tracking_rad=review['max_tracking_rad'],peak_current_ma=review['peak_current_ma'],min_voltage_v=review['min_voltage_v'],
    cube_placement_visually_matches_requested_finger_support=True,no_visible_human_cube_contact_in_named_frames=True,
    human_hand_visible_at_robot_base=True,cube_removed_operator_confirmed=True,all_torque_off_verified=True,settings_restored=True,
    camera_cube_hand_registration_verified=False,physical_grasp_verified=False,physical_ninety_degree_rotation_verified=False,
    repeat_capture_required=False,assistant_motor_goal_writes=0,assistant_torque_changes=0)
(OUT/'observation_status.json').write_text(json.dumps(details,ensure_ascii=False,indent=2),encoding='utf-8')
for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json'):
    p=ROOT/name;r=json.loads(p.read_bytes());r.update(timestamp=datetime.datetime.now().astimezone().isoformat(),
        new_start_cube_placement_complete=True,placement_observation=details,next_user_command=None,next_user_command_ready=False,
        next_action='Offline quarter-turn path validation using completed actual placement. Do not repeat placement capture. No new physical action requested yet.',
        reverse_loaded_execution_ready=False,physical_ninety_degree_rotation_verified=False)
    p.write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
text=f'''## CURRENT 2026-10-25 07:54: 새 시작 자세의 큐브 배치 촬영 완료

- `reverse_start_placement_20261025_075405_164917`에서 복귀, PLACE 촬영, REMOVE 확인과 OFF 및 설정 복원이 완료됐다. 최대 추종 {review['max_tracking_rad']:.3f}rad, 최대 전류 {review['peak_current_ma']:.0f}mA, 최저 전압 {review['min_voltage_v']:.1f}V다. 실제 회전은 실행하지 않았다.
- 빈손과 배치 및 고정 유지 사진을 직접 확인했다. 큐브가 검지와 중지 쪽 평평한 윗면에 받쳐져 있고, 두 배치 사진에서 사람 손은 큐브에 닿아 보이지 않는다. 사람 손은 로봇 본체에 닿아 보여 모터 보정이나 자유 공간 증거로 사용하지 않는다. 큐브 제거는 사용자 SPACE 확인과 별도 OFF 기록에 근거한다. 정밀한 카메라와 손의 3차원 보정을 완료했다는 주장은 하지 않는다.
- 이전 배치 촬영 요청은 충족됐다. 같은 사진 촬영을 반복 요청하지 않는다. 현재 다음 사용자 작업은 없으며 새 배치에 사용할 전체 90도 경로를 오프라인 검증한다. 실제 목표 및 토크 쓰기는 이번 검토에서 0회다. 전체 회전 실행은 아직 준비되지 않았다. 전시 문구는 변경하지 않았다.

<!-- END OBSERVED PLACEMENT 075405 -->

'''
for name in ('sac_current_status.md','professor_progress_20261015.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8')
    if old.startswith('## CURRENT 2026-10-25 07:54:'):
        old=old.split('<!-- END OBSERVED PLACEMENT 075405 -->',1)[1].lstrip()
    p.write_text(text+old.replace('## CURRENT','## PREVIOUS'),encoding='utf-8')
print(json.dumps(details,ensure_ascii=False))
