"""Record saved-pair analysis and the readiness UI change; no real devices."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_pose_readonly_20261023_165647_488518'


def main():
    pair=json.loads((FOLDER/'paired_capture_review.json').read_bytes())
    tracking=json.loads((FOLDER/'palm_template_tracking.json').read_bytes())
    assert pair['pair_count']==pair['stable_endpoints']==12
    assert pair['max_pair_drift_ticks']==1 and pair['all_torque_off']
    assert pair['palm40_recovered_frames']==0 and not tracking['hypotheses']
    assert pair['source_sha256']==tracking['source_sha256']
    from grasp_place_plan import GraspPlacePlan
    from thumb_live_start_gate import validate_proof
    previous=ROOT/'cube_grasp_trial_20261023_044607_477240'
    proof=validate_proof(GraspPlacePlan(previous/'fresh_off_request.json'),previous/'current_start_screen.json')
    files=('cube_pose_readonly_capture.py','fixed_exposure_camera.py',
           'cube_marker_readiness.py','test_cube_pose_readonly_capture.py')
    evidence=dict(status='paired_encoders_ok_palm_glare_unresolved_live_marker_feedback_ready',
        source=FOLDER.name,source_sha256=pair['source_sha256'],pairs_saved=12,all_torque_off=True,
        max_endpoint_drift_ticks=1,min_voltage_v=pair['min_voltage_v'],
        cube_marker_decoded_frames=12,palm_marker_decoded_frames=0,
        saved_frame_roi_decoding_attempts=pair['attempts'],palm_roi_recovered_frames=0,
        historical_template_tracking_accepted_poses=0,
        best_multi_template_corner_disagreement_px=min(r['maximum_reference_corner_disagreement_px']
            for r in tracking['tracking_rows'] if r['maximum_reference_corner_disagreement_px'] is not None),
        maximum_accepted_corner_disagreement_px=2.,old_q_reused=False,
        no_unverified_pose_exported=True,no_motor_targets_changed=True,
        current_marker_to_model_geometry_verified=False,
        readiness_requirement='3 consecutive fresh, same-frame decoded ID40 + cube marker, each minimum edge >=24px',
        one_terminal_space_latched=True,readiness_wait_limit_s=30.,
        preview_overlays_preserve_raw_pixels=True,
        device_free_tests_passed=39,
        test_command='.runtime\\python\\python.exe -X utf8 -B -m unittest test_cube_pose_readonly_capture test_camera_exposure_sweep test_camera_preview_isolation test_camera_readonly_check',
        existing_motor_proof_matches=True,static_cases=len(proof['static_screens']),dynamic_cases=len(proof['dynamic_sensitivity']),
        code_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files},
        next_launcher='leap_cube_pose_readonly.cmd',
        next_user_action='Leave cube/fingers unchanged; shade glare on palm ID40 or adjust camera until live green READY, then terminal SPACE once.',
        servo_writes_sent=0,real_camera_or_serial_opened_by_review=False)
    (FOLDER/'marker_readiness_ready_review.json').write_text(json.dumps(evidence,indent=2),encoding='utf-8')
    # Keep the project handoff in Korean, separate from completion-assumed panel copy.
    title='## CURRENT 2026-10-23 16:56: 사진과 각도 12쌍 저장 / ID40 반사 미해결'
    note='''

- `cube_pose_readonly_20261023_165647_488518`에서 사진과 관절 전후 읽기 12쌍을 저장했다. 16개 토크는 모두 OFF였고, 관절의 전후 최대 차이는 1틱, 최저 전압은 4.6V다. 큐브 ID5는 12장 모두 직접 해독됐으나 손바닥 ID40은 0장이다. 원래 카메라 노출과 자동 getter 복원도 확인했다. 이번에는 전원이나 통신 문제가 아니다.
- 저장 사진의 손바닥 영역에서 원본 회색조, 색 채널, 대비 처리와 두 가지 테두리 허용 설정을 사용한 168회 해독을 시도했지만 ID40은 복구되지 않았다. 이전 직접 해독된 4개 손바닥 사진을 기준으로 평면 특징 추적도 수행했으나, 여러 기준 사진 사이의 모서리 차이가 최소 2.75px로 설정한 2px 이내 조건을 통과하지 못했다. 예전 관절 값을 가져오거나 이를 새 직접 해독으로 표시하지 않았다. 새 실물 경로의 근거로 내보낸 위치는 없다.
- 새 읽기 전용 촬영에서는 직접 해독된 마커를 화면에 그리고, ID40 반사 또는 큐브 마커 크기 문제를 즉시 안내한다. 같은 프레임의 ID40과 큐브 마커가 각각 짧은 변 24px 이상으로 연속 3번 읽힐 때 READY를 표시한다. 터미널 SPACE 한 번을 기억하고 최대 30초 동안 인식을 기다린다. 모터를 켜는 게이트가 아니라 사진 인식 확인이며 서보 PING/READ만 사용한다.
- 원본 픽셀 보존, 유한 대기, SPACE 한 번 처리와 기존 읽기/카메라 검사 총 39개 통과. 기존 실물 모터 실행본 및 공유 카메라 코드는 변경하지 않았고 04:46 실행 증명의 67정적/3응답 검증도 일치한다.
- **다음 사용자 작업:** 손과 큐브를 그대로 두고 같은 `leap_cube_pose_readonly.cmd`를 실행한다. 조명에 그늘을 만들어 ID40의 흰 반사를 줄이거나 카메라를 조금 움직여 ID40과 ID5의 녹색 테두리 및 초록 READY를 확인한 뒤 터미널 SPACE 한 번. ID40이 보이지 않는 채 반복 촬영하지 않는다. 새 결과를 읽어 다음 모델 비교를 진행한다.
- 근거: 이 실행 폴더의 `paired_capture_review.json`, `palm_template_tracking.json`, `marker_readiness_ready_review.json`. 전시용 족자는 사용자가 요청한 완성 시점 가정 문구이며 실제 개발 완료 증거로 사용하지 않는다.

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith(title):continue
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(title+note+old,encoding='utf-8')
    print(json.dumps({k:v for k,v in evidence.items() if k!='code_sha256'},indent=2))


if __name__=='__main__':main()
