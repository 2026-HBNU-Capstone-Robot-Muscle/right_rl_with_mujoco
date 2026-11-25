"""Record the completed physical observation and retire its next-user command."""
import datetime as dt,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'new_pair_grasp_20261027_050946_894527'
def main():
    path=FOLDER/'visual_review/review.json';r=json.loads(path.read_bytes());stamp=dt.datetime.now().astimezone().isoformat();state=dict(status='physical_loaded_grasp_open_observation_complete',timestamp=stamp,latest_user_trial=r['source'],latest_review=path.relative_to(ROOT).as_posix(),actual_grasp_open_video=r['video'],new_pair_grasp_observation_complete=True,new_pair_grasp_operator_ready=False,physical_cube_lift_or_clamp_force_verified=False,physical_horizontal_90_verified=False,horizontal_quarter_turn_verified=False,arbitrary_direction_rotation_verified=False,physical_forward_flip_successes=2,next_user_command=None,next_user_task=None,next_user_action=None,next_user_command_ready=False,next_motion_command_ready=False,rotation_execution_ready=False,pending_user_input=None,next_action='Use the reached grasp/contact and actual encoder data for offline horizontal90 trajectory work. Do not repeat this grasp-only observation.',assistant_motor_writes=0,camera_opened_this_turn=False,serial_ports_opened_this_turn=False)
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261026/fresh_empty_start_20261026_235258/progress_20261027.json'):
        p=ROOT/name;old=json.loads(p.read_bytes());old.update(state);p.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block=f'''## CURRENT 2026-10-27: 실물 큐브 파지 접근과 자동 열기 완료

- `new_pair_grasp_20261027_050946_894527`이 v2 경로로 중단 없이 완료됐다. 실제 큐브 배치 후 접근605목표, 유지40목표(명목3초), 자동열기605목표, 제거 확인과 토크OFF/설정 복구가 모두 기록됐다. 건강 읽기 {r['health_samples']:,}개, 최소전압 {r['minimum_voltage_v']:.1f}V, 최대전류 {r['peak_current_ma']}mA, 최대추종 {r['maximum_tracking_rad']:.5f}rad, 전압 중단0회다. 영상 {r['camera_frames']:,}장, 검은 프레임0장이다.
- 저장 사진과 단계별 영상에서 엄지 패드가 큐브 앞면에 접근/접촉하고, 중지가 뒤쪽으로 감싼 뒤 다시 열리는 모습을 확인했다. 큐브는 받침과 뿌리 위에서 유지됐다. 들기나 압착력을 검증한 실행은 아니므로 강한 파지 성공으로 확정하지 않는다. 사람 손은 본체/받침 쪽에 보이며 선택 프레임에서는 회전 중 큐브를 잡은 모습이 없다. 90도 회전은 명령하지 않았다.
- 배치 전과 유지 사진에는 큐브ID3과 고정ID41만 함께 검출됐다. 종료 사진은ID0/3/41이 검출됐지만 같은 두 큐브 마커의 전후 쌍이 없어 기존 상대회전 계측 조건을 만족하지 않았다. 보이는 작은 방향 변화에 정확한 각도나90도 성공을 부여하지 않는다.
- 원본 프레임의 읽기 시각으로 재생한 실물 파지/열기 영상은 `{r['video']}`이다. `visual_review/grasp_open_contact_sheet.jpg`와 `review.json`에 사진과 통계를 보존했다. 실물 수직 뒤집기 성공 총2회는 유지하고 수평90도는 계속 미검증이다.
- 같은 파지 전용 실행을 다시 요청하지 않는다. 실제 끝 관절값과 새 접촉 형태를 출발점으로 수평90도 경로를 오프라인 계산 중이다. 다음 사용자 명령은 비웠고 새 치수/촬영/로그 복사를 요구하지 않는다. 모터OFF 뒤 엄지CMC가1723틱까지 내려온 불안정 끝 상태를 그대로 안정된 다음 출발점으로 쓰지 않는다. 다음 실제 복귀는 새 읽기와 기존 단조 재진입 모델 검사가 필요하다.

<!-- END PHYSICAL PAIR OBSERVATION 20261027 -->

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        p=ROOT/name;old=p.read_text(encoding='utf-8-sig');marker='<!-- END PHYSICAL PAIR OBSERVATION 20261027 -->'
        if marker in old:raise RuntimeError('Already recorded')
        p.write_text(block+old.replace('## CURRENT 2026-10-27:','## PREVIOUS 2026-10-27:',1),encoding='utf-8')
    print('Recorded completed physical observation. No next-user command or device access.')
if __name__=='__main__':main()
