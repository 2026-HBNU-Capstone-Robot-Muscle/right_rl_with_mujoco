"""Retain the false-stop evidence and record the tested temporal correction."""
import hashlib
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from empty_start_marker_check import make_empty_detector,inspect_empty_frame,EmptyStartEvidence

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'slow_grasp_demo_20261023_180831_999579'


def main():
    blob=(FOLDER/'result.json').read_bytes();source=json.loads(blob)
    validation_path=ROOT/'right_hand_rotation_20261023/slow_grasp_demo_validation.json'
    validation=json.loads(validation_path.read_bytes())
    native=json.loads((ROOT/'right_hand_rotation_20261023/demo_empty_start_native_review.json').read_bytes())
    assert validation['passed'] and validation['tests_run']==224
    assert source['all_torque_off_verified'] and not source.get('enabled_at') and not source.get('samples')
    assert all(s['accepted'] for s in native['fixtures'])
    rows=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    detector=make_empty_detector();evidence=EmptyStartEvidence();candidate_frames=confirmed_frames=0
    for row in rows:
        frame=cv.imdecode(np.frombuffer((FOLDER/'camera'/row['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
        candidates=inspect_empty_frame(frame,detector)
        result=evidence.observe(row['camera_read_monotonic'],candidates)
        candidate_frames+=bool(candidates);confirmed_frames+=bool(result['confirmed_cube_ids'])
    assert len(rows)==245 and confirmed_frames==0
    record=dict(status='empty_hand_false_stop_reviewed_and_temporal_gate_tested',source=FOLDER.name,
        source_sha256=hashlib.sha256(blob).hexdigest(),
        original_stop_live_decoded_ids=source['empty_start_marker_screen']['decoded_cube_ids'],
        original_stop_did_not_save_exact_raw_frame=True,
        exact_live_error_pixel_location_verified=False,
        saved_empty_frames_reviewed=len(rows),new_exact_detector_candidate_frames=candidate_frames,
        new_confirmed_cube_frames=confirmed_frames,
        actual_saved_cube_sequence_still_confirmed=True,
        automatic_off_stop_requires_same_id_and_location=True,
        minimum_confirmation_frames=3,minimum_confirmation_seconds=.2,minimum_marker_edge_px=20,
        single_transient_detection_stops_off_screen=False,
        exact_codeword_required=True,multi_pass_pose_tracker_used_for_empty_stop=False,
        next_confirmed_stop_saves_exact_raw_png=True,
        no_marker_decoding_during_configuration_or_motion=True,
        physical_cube_rotation_verified=False,physical_demo_completed=False,
        motor_goal_or_drive_limit_changes=False,torque_off_verified=True,
        no_motor_configuration_or_motion_started_in_source_run=True,
        regression_tests=validation['tests_run'],native_fixtures=native['fixtures'],
        validation_sha256=hashlib.sha256(validation_path.read_bytes()).hexdigest(),
        next_launcher='leap_slow_grasp_demo.cmd',
        next_user_action='Keep hand empty and cube outside camera until PLACE CUBE, then follow existing SPACE prompts.',
        real_camera_or_motor_port_opened_by_assistant=False)
    destination=ROOT/'right_hand_rotation_20261023/demo_false_alarm_fix_review.json'
    destination.write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    title='## CURRENT 2026-10-23 18:08 실행 검토: 빈손 오인식 수정'
    note='''

- `slow_grasp_demo_20261023_180831_999579`는 시뮬레이션 대기 중 새 빈손 검사에서 중단됐다. 실제 모터 설정, 토크 활성화와 목표 이동은 시작하지 않았고 16개 토크 OFF를 확인했다. 카메라 기존 설정도 복원했다. `settings_restored=false`는 이 실행에서 모터 설정 변경 및 복원 단계에 들어가지 않았다는 사실과 함께 해석한다.
- 저장된 시작과 마지막 사진은 큐브가 없는 손이다. 원래 감지기는 한 프레임의 허용적인 해독으로도 실행을 종료했다. 당시 live 기록은 ID5다. 정확한 원본 프레임을 별도로 보존하지 않아 해당 픽셀 위치는 확정하지 않는다. JPEG 245장 재생에서는 ID3가 한 프레임에서 손가락 사이 틈을 잘못 잡았다. live ID5와 JPEG 재생 ID3를 같은 검출이라고 취급하지 않는다.
- 빈손 검사에는 자세 추정용 다중 대비 및 조밀 임계값 감지기를 사용하지 않는다. 별도 정확한 코드 해독과 검은 테두리 조건 및 최소 변 길이 20px을 적용했다. OFF 경로 검사와 카운트다운 중 자동 종료에는 같은 ID와 위치가 새 프레임 3장 이상 및 0.2초 이상 이어져야 한다. 한 번의 후보는 종료시키지 않고 다음 새 프레임에서 다시 확인한다. 첫 SPACE는 후보가 없어지고 새 프레임 10장 동안 해독이 없으면 받는다. 마커 부재가 빈손의 증명은 아니므로 실제 큐브 제거는 필요하다.
- 이번에 저장된 빈손 245장 전체에서 새 검사로 확인된 큐브는 0건이다. 이전 실제 큐브가 올라간 영상에서는 ID5가 연속 확인돼 차단 기능이 유지된다. 앞으로 확인된 큐브 때문에 중단하면 정확한 원본 PNG와 시간 및 모서리 좌표를 저장한다. 모터 설정과 동작 중에는 해독을 추가하지 않으며 PLACE 뒤에는 ID40 가림을 중단 조건으로 쓰지 않는다.
- 장치 없는 회귀 검사 224개 통과. 기존 시작과 이번 18:08 실제 OFF 관절값으로 각각 67정적 및 3응답의 MuJoCo 경로 검사를 새 코드로 수행했다. 기존 21개 증명은 현재 해시 및 내용이 일치하는 것을 검증해 재사용했다. 경로 목표, 전류, PID 및 실제 상태 중단 기준은 바꾸지 않았다.
- **지금 사용자 작업:** 손가락은 그대로 두고 큐브를 손과 카메라 화면 밖으로 둔다. 같은 `leap_slow_grasp_demo.cmd`로 실행한다. 빈손 첫 SPACE, 약지 분리 확인 SPACE, PLACE CUBE 때 큐브 배치와 손 제거 후 SPACE, REMOVE CUBE 때 큐브 제거 후 SPACE. 끝나면 했어만 알려주면 된다. 추가 카메라 촬영이나 전체 로그 붙여넣기를 요구하지 않는다.
- 근거: `right_hand_rotation_20261023/demo_false_alarm_fix_review.json`, `demo_empty_start_native_review.json`, `slow_grasp_demo_validation.json`, 이번 실행 폴더의 `empty_start_false_alarm_review.json`, `leap_slow_grasp_demo_README.md`.

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith(title):continue
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(title+note+old,encoding='utf-8')
    print(json.dumps(record,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
