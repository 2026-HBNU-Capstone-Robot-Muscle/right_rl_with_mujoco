"""Record evidence from the interrupted trial without changing raw results."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FOLDER = ROOT / 'grasped_yaw_trial_20261023_215230_227769'


def main():
    path = FOLDER / 'interrupted_yaw_review.json'
    review = json.loads(path.read_bytes())
    assert review['all_torque_off_verified'] and review['settings_restored']
    assert review['turn_samples_before_stop'] == 19 and not review['full_turn_complete']
    assert len(review['read_failures']) == 2
    review.update(
        visual_verdict='No obvious cube rotation visible in the saved baseline versus approximately 1.1 seconds of partial turn. Full turn and after-hold were not reached.',
        visual_rotation_angle_measured=False,
        exposure_cause='The camera original getter was -5 and the trial applied -7. Saved frames are visibly dark; no before/after original-exposure image pair exists in this run.',
        motor_id_15_failure_not_proven=True,
        runtime_or_motion_limits_changed=False,
        next_user_command='leap_camera_exposure_sweep.cmd',
        next_capture_reason='Compare original camera appearance and reversible manual exposure profiles without moving the robot.',
        camera_diagnostic_installs_runtime_settings=False,
        additional_motor_trial_requested=False)
    path.write_text(json.dumps(review, ensure_ascii=False, indent=2), encoding='utf-8')
    title = '## CURRENT 2026-10-23 21:52: 파지 후 작은 회전 중 통신 중단 / 촬영 확보'
    note = '''

- 실제 실행은 `grasped_yaw_trial_20261023_215230_227769`다. 준비와 큐브 배치 및 파지 접근 322개 목표, 고정 유지 60개 표본을 완료했다. 작은 회전은 계획 90개 중 19개의 건강한 읽기 표본까지만 남았고 마지막 약 1.11초 동안의 부분 동작 뒤 중단됐다. 고정 후 관찰과 REMOVE 단계에 도달하지 않았으므로 완주나 실제 목표 회전 성공을 주장하지 않는다.
- 첫 동기 읽기는 ID1~5 뒤 패킷 손상 -3002로 47ms에 실패했다. 기존의 읽기 전용 복구 1회를 수행했으나 두 번째 읽기도 ID1~14 뒤 ID15 응답 시간 초과 -3001로 총 94ms에 실패했다. 오류 단계에서 새 목표를 진행하지 않고 OFF로 종료했다. 첫 누락 ID가 달라 특정 모터 ID15의 고장이나 기계적 걸림으로 단정할 수 없다. 통신 복구 횟수나 건강 검사 기준을 바꾸지 않았다.
- 측정 최소 전압 4.0V, 최대 절댓값 전류 96mA, 최대 추종 오차 약 0.07363rad. 모든 토크 OFF와 설정 복원, 카메라 원래 노출 -5 복귀가 확인됐다. `cube_present=true`는 정상 REMOVE 확인을 받지 못했다는 결과다. 새로운 빈손 실행 전에는 큐브를 제거한다.
- 원본 카메라 프레임 1049장을 확보했고 검은 화면 판정은 0장이었다. 저장 화면은 어둡고 손의 검은 부분을 구분하기 어렵다. 실행은 원래 노출 getter -5에서 -7을 적용했다. 이 실행에는 원래 노출의 대응 사진이 없으므로 초점 변경이나 하드웨어 화질 고장을 단정하지 않는다. 21:52:04 미리보기 중 노출 getter -1로 바뀐 별도 실행은 모터 접근 전에 중단됐으며 최종 실행의 기록과 섞지 않는다.
- 마지막 건강한 ON 읽기 시각 이전의 원본만 사용해 `before_interrupted_turn_contact_sheet.jpg`와 `grasp_then_interrupted_yaw.mp4`를 만들었다. 영상은 약 4.27초이며 원래 읽기 시간 간격을 따른다. 고정 유지 대비 부분 회전에서 뚜렷한 큐브 회전은 보이지 않는다. OFF 뒤 처짐이나 큐브 제거 움직임을 성공 증거로 쓰지 않았다. 원본 결과와 이미지 및 모터 경로는 수정하지 않았다.
- **지금 사용자 작업:** 모터 전원을 끄고 큐브를 로봇 손 밖 탁자에 놓는다. `leap_camera_exposure_sweep.cmd`를 실행해 카메라에 손바닥 ID40과 큐브 초록 중앙 ID5를 함께 보여주고 SPACE 한 번을 누른다. 이 기존 진단은 카메라만 사용해 원래 설정과 수동 노출을 비교 저장한 뒤 원래 설정을 복원하며, 로봇 구동 설정을 설치하지 않는다. 마커가 완전히 인식되지 않아도 저장된 밝기 비교를 검토할 수 있다. 끝나면 했어만 알리면 된다. 모터 회전 재시험은 아직 요청하지 않는다.
- 사용자에게 요청한 화질 유형 질문은 답이 없으므로 어둡다는 저장 화면 관찰만으로 진행한다. 연결 복구 후 카메라 장치명은 Streaming Webcam, 실제 크기는 1280×720 MJPG였다. 임의로 다른 장치로 전환하지 않는다.
- 근거: `grasped_yaw_trial_20261023_215230_227769/interrupted_yaw_review.json`, 원본 `result.json`, `camera/frames.json`. 검토 과정에서 실제 카메라나 모터를 열지 않았다.

'''
    for name in ('sac_current_status.md', 'professor_progress_20261015.md'):
        file = ROOT / name
        old = file.read_text(encoding='utf-8')
        if old.startswith(title):
            continue
        if old.startswith('## CURRENT '):
            old = old.replace('## CURRENT ', '## PREVIOUS ', 1)
        file.write_text(title + note + old, encoding='utf-8')
    print(json.dumps({key: review[key] for key in ('status', 'visual_verdict', 'camera_restored',
        'all_torque_off_verified', 'settings_restored', 'next_user_command')}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
