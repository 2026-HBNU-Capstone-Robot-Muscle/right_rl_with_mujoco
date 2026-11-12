"""Record the camera-only runtime revision after native/device-free checks."""
import hashlib
import json
from pathlib import Path
from grasped_yaw_plan import GraspedYawPlan
from slow_grasp_demo_plan import SlowGraspDemoPlan
from thumb_live_start_gate import validate_proof

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261023'


def main():
    validation=json.loads((OUT/'grasped_yaw_validation.json').read_bytes())
    assert validation['passed'] and validation['tests_run']==238
    assert validation['previous_native_fixtures_passed']==21
    exposure=json.loads((OUT/'yaw_exposure_review.json').read_bytes())
    assert exposure['selected_exposure']==-5. and not exposure['simultaneous_palm_cube_pose_registration_verified']
    records=[]
    for label in ('grasped_yaw_fixture','grasped_yaw_prior_fixture','grasped_yaw_open_fixture',
                  'grasped_yaw_after_stop_fixture','slow_grasp_demo_fixture'):
        folder=OUT/label;kind=SlowGraspDemoPlan if label=='slow_grasp_demo_fixture' else GraspedYawPlan
        plan=kind(folder/'fresh_off_request.json');proof=validate_proof(plan,folder/'native_screen.json')
        prior=[json.loads(path.read_bytes()) for path in folder.glob('before_grasped_yaw_*.json')]
        if label!='grasped_yaw_after_stop_fixture':
            assert any(row.get('accepted') and row.get('plan_sha256')==proof['plan_sha256'] for row in prior)
        records.append(dict(fixture=label,static_cases=len(proof['static_screens']),
            dynamic_cases=len(proof['dynamic_sensitivity']),
            maximum_tracking_rad=max(row['max_tracking_rad'] for row in proof['dynamic_sensitivity']),
            proof_sha256=hashlib.sha256((folder/'native_screen.json').read_bytes()).hexdigest(),
            prior_goal_digest_unchanged=label!='grasped_yaw_after_stop_fixture'))
    replay=json.loads((OUT/'grasped_yaw_after_stop_fixture/replay_review.json').read_bytes())
    assert replay['accepted'] and replay['saved_endpoint_is_not_live_start_approval']
    record=dict(status='brighter_yaw_trial_ready_new_fresh_OFF_screen_still_required',
        selected_camera_exposure=-5.,legacy_default_camera_exposure=-7.,tests_passed=238,
        native_start_checks=records,old_native_fixtures_validated=21,
        old_native_proofs_reused_only_after_exact_validation=True,
        motion_goal_digests_unchanged=True,drive_and_health_limits_changed=False,
        read_timeout_recovery_policy_changed=False,
        raw_execution_results_changed=False,physical_rotation_verified=False,live_sac=False,
        palm_registration_verified=False,assistant_opened_real_devices=False,
        source_exposure_review_sha256=hashlib.sha256((OUT/'yaw_exposure_review.json').read_bytes()).hexdigest(),
        next_user_command='leap_grasped_yaw_trial.cmd',further_camera_sweep_required=False)
    (OUT/'yaw_camera_revision_ready_review.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    title='## CURRENT 2026-10-23 22:03: 밝기 비교 완료 / 회전 시험 카메라 -5 적용'
    note='''

- 사용자는 앞의 세 촬영에는 없었고 다시 했다고 정정했다. 이 진술을 보존하고 앞 세 촬영의 조건을 추정해 현재 설정 선택에 쓰지 않는다. 새 결과 `camera_exposure_sweep_20261023_220318_878174`를 기준으로 검토했다. 큐브는 손 밖 오른쪽 탁자에 있고, 초록 윗면과 주황 앞면 마커가 보인다. 사진의 손바닥 ID40에는 반사광이 남아 있다.
- 새 자료에서 ID5는 원래 -5와 수동 -5 및 -6에서 각각 6/6장, -7에서 5/6장 검출됐다. ID40은 24장 모두 미검출이다. `Readable palm+cube pair found: False`는 이 쌍의 동시 검출 실패이며, 촬영 실패나 전체 큐브 미검출을 의미하지 않는다. 원본 24장과 노출 복원은 정상 저장됐다. 이 자료에서 3차원 등록 성공을 주장하지 않는다.
- 이번 회전 시험 `GraspedYawPlan`만 -5 고정 노출을 사용하도록 `GraspedYawCamera`를 지정했다. 검은 손이 더 밝게 보이는 설정이며 전체 평균 회색값은 -5 약 62.1, -6 약 39.7, -7 약 19.7이다. 기존 느린 파지와 읽기 전용 기본값 -7은 그대로다. worker가 명시된 측정 프로필을 받고 설정 읽기 확인 및 실행 중 유지, 종료 시 원래 설정 복원을 수행한다. 지원하지 않거나 바뀌면 중단한다. 초점이나 게인은 변경하지 않았다.
- 이번 변경은 파지나 회전의 목표 경로를 바꾸지 않는다. 세 회전 시작과 느린 파지 시작의 새 네이티브 증명을 만들었고, 이전 저장 증명과 목표 digest가 같음을 확인했다. 21개 다른 경로의 기존 증명도 정확한 요청 및 현재 코드 검증을 통과해 재사용했다. 추가로 실제 21:52 통신 중단 뒤 저장된 OFF 끝점에서도 67개 정적 및 3개 응답 검사를 통과했다. 그 OFF 표본은 양 끝 안정 상태가 아니었으므로 현재 실물 자세의 승인으로 사용하지 않으며, 실행 시 안정된 새 읽기와 경로 검사를 다시 수행한다.
- 장치 없는 검사 238개가 통과했다. 새 검사에는 -5 설정과 복원, 지원하지 않는 값, Q 및 카메라 읽기 실패, 노출 변동, 실제 저장 밝은 빈손 반사 이미지와 큐브 이미지의 시작 차단을 포함한다. 이전 마커 가림 및 통신과 전류, 전압 및 추종 중단 조건은 유지한다. 읽기 재시도 횟수를 늘리지 않았고, 실제 장치는 에이전트가 열지 않았다.
- **지금 사용자 작업:** 큐브를 손과 카메라 화면 밖으로 빼고, 다른 제어 프로그램을 닫는다. 앞선 통신 오류가 있었으므로 모터 전원을 끈 상태에서 USB 및 손 케이블 접속이 느슨하지 않은지 확인한다. 전원을 연결해 `leap_grasped_yaw_trial.cmd`를 실행한다. 빈손 첫 SPACE, 약지 분리 확인 SPACE, PLACE에서 초록 위와 주황 카메라 쪽으로 놓고 손을 뺀 뒤 SPACE. REMOVE 전까지 만지지 않는다. 큐브 제거 후 SPACE이며, 걸리면 Q로 OFF한 뒤 제거한다. 추가 밝기 촬영은 요청하지 않는다. 끝나면 했어만 알리면 원본을 검토한다.
- 실제 원하는 방향 회전은 계속 미검증이며, 직전 21:52 실물 실행은 회전 19/90 표본에서 통신 오류로 중단됐다. 본 변경은 노출 문제에 대응하는 수정으로, 통신 문제가 해결됐다고 주장하지 않는다.
- 근거: `right_hand_rotation_20261023/yaw_exposure_review.json`, `yaw_camera_revision_ready_review.json`, `grasped_yaw_validation.json`, `grasped_yaw_after_stop_fixture/replay_review.json`. 원본 실행 결과나 후보 경로 파일은 수정하지 않았다.

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        file=ROOT/name;old=file.read_text(encoding='utf-8')
        if old.startswith(title):continue
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        file.write_text(title+note+old,encoding='utf-8')
    print(json.dumps(record,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
