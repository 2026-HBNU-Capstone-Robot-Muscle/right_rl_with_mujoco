"""Review the stored 16:53 attempt; no camera/serial/simulation access."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_pose_readonly_20261023_165353_350016'


def main():
    blob=(FOLDER/'result.json').read_bytes();data=json.loads(blob)
    assert data['status']=='stopped' and 'comm=-3001' in data['error']
    assert 'ID1:' in data['error'] and not data.get('pairs')
    assert data['servo_write_commands_sent']==data['torque_changes_requested']==0
    assert data['camera_restoration_verified']
    assert not list(FOLDER.glob('pair_*.png'))
    record=dict(status='camera_worked_motor_id1_no_response_before_capture',
        source_sha256=hashlib.sha256(blob).hexdigest(),source_timestamp=data['timestamp'],
        camera_exposure_supported=data['camera_metadata']['exposure_profile']['supported'],
        camera_exposure_readback=data['camera_metadata']['exposure_profile']['readback']['exposure'],
        camera_preview_frames=data['camera_worker']['frames_sent'],
        camera_original_settings_restored=data['camera_restoration_verified'],
        motor_error=data['error'],motor_response_received=False,torque_state_verified=False,
        complete_encoder_pairs=0,raw_pair_images=0,old_q_reused=False,
        source_off_endpoints_flag_is_vacuous=True,
        next_capture_code_requires_nonempty_completed_pairs_for_off_flag=True,
        next_capture_code_has_motor_ping_phase_and_power_cable_hint=True,
        device_free_tests_passed=35,servo_write_commands_sent=0,real_devices_opened_by_review=False,
        next_user_action='Check separate motor power and USB/adapter-to-hand cable, close other controllers, rerun the same READ ONLY launcher and press terminal SPACE once. Leave cube/fingers as they are.')
    (FOLDER/'connection_review.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    title='## CURRENT 2026-10-23 16:53: 카메라 정상 / ID1 응답 없어 동시 촬영 전 중단'
    note='''

- `cube_pose_readonly_20261023_165353_350016`은 COM7을 열었지만 ID1 PING에서 `model=0, comm=-3001, error=0`으로 중단했다. 응답을 받지 못했으므로 실제 모델이나 토크 상태를 확인한 것으로 해석하지 않는다. 모터 전원 OFF·케이블·통신 중 어느 원인인지는 저장 자료만으로 확정할 수 없다.
- 카메라 이름·720p MJPG·노출 −7 적용은 정상이며 미리보기 103프레임이 전달됐다. 원래 노출 −5/자동 getter −1 복원도 확인했다. PING 단계에서 중단해 원본 사진/관절 쌍은 0개이며 새 큐브 위치·접촉 계산은 수행할 수 없다. 예전 관절 값을 재사용하지 않는다. 목표·토크 변경 명령은 0개다.
- 무자료에서 OFF 확인을 참으로 기록하던 빈 목록 판정을 새 촬영 코드에서 수정했다. 완료 쌍이 있어야 OFF 끝점 확인을 참으로 표시한다. 실패 단계 및 별도 모터 전원/케이블 안내도 추가했다. 장치 없는 기존/새 검사 35개 통과. 기존 모터 실행본과 공유 카메라 코드는 변경하지 않았다.
- **다음 사용자 작업:** USB와 별도 5V 모터 전원을 연결하고 어댑터→손 케이블/다른 제어 프로그램 종료를 확인한다. 현재 손·큐브는 그대로 두고 같은 `leap_cube_pose_readonly.cmd`를 실행해 터미널 SPACE 한 번. 현재 막힌 지점은 촬영 구도가 아니라 모터 응답이다.
- 근거: 실행 폴더 `connection_review.json` 및 원본 `result.json`. 사용자 확인 전에는 모터 전원 OFF였다고 단정하거나 실물 동작을 반복하지 않는다.

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith(title):continue
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(title+note+old,encoding='utf-8')
    print(json.dumps(record,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
