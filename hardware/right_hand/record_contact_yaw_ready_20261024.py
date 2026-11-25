"""Record verified offline work and the next operator action, without devices."""
import hashlib
import json
from pathlib import Path
from grasped_yaw_plan import GraspedYawPlan,CANDIDATE,CANDIDATE_SHA
from revalidate_contact_yaw_20261024 import OLD_FIXTURES,LATEST
from thumb_live_start_gate import validate_proof

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024'
PHYSICAL=ROOT/'grasped_yaw_trial_20261023_234707_886202'


def main():
    regression=json.loads((OUT/'contact_yaw_regression_validation.json').read_bytes())
    integration=json.loads((OUT/'contact_yaw_integration_validation.json').read_bytes())
    assert regression['passed'] and regression['tests_run']>=252
    assert not regression['errors'] and not regression['failures']
    assert integration['passed'] and integration['all_seven_paths_replayed']
    assert regression['candidate_sha256']==integration['candidate_sha256']==CANDIDATE_SHA
    assert hashlib.sha256(CANDIDATE.read_bytes()).hexdigest()==CANDIDATE_SHA
    proofs=[]
    for folder in OLD_FIXTURES+(LATEST,):
        proof=validate_proof(GraspedYawPlan(folder/'fresh_off_request.json'),folder/'native_screen.json')
        proofs.append(dict(fixture=str(folder.relative_to(ROOT)),accepted=proof['accepted'],
            static_cases=len(proof['static_screens']),dynamic_cases=len(proof['dynamic_sensitivity']),
            maximum_tracking_rad=max(v['max_tracking_rad'] for v in proof['dynamic_sensitivity'])))
    raw=(PHYSICAL/'result.json').read_bytes();sha=hashlib.sha256(raw).hexdigest()
    assert sha=='adc0cf67dfb20d1e0de6127b8d5961fa19beae6173d657c06fa96b65dc58948c'
    physical=json.loads(raw)
    assert physical['rotation_command_path_completed'] and physical['failed_phase']=='remove_cube'
    assert physical['all_torque_off_verified'] and physical['settings_restored']
    motion=json.loads((PHYSICAL/'cube_plane_optical_flow.json').read_bytes())
    assert motion['all_source_frames_before_removal'] and not motion['physical_target_yaw_verified']
    review_path=PHYSICAL/'completed_yaw_review.json'
    review=json.loads(review_path.read_bytes())
    assert review['source_sha256']==sha
    review.update(visual_verdict='lateral_cube_motion_visible_target_five_degree_yaw_not_verified',
        image_motion_diagnostic='cube_plane_optical_flow.json',
        cube_image_translation_px=motion['top_face_tracking']['image_translation_median_px'],
        static_palm_image_translation_px=motion['static_palm_tracking']['image_translation_median_px'],
        exact_physical_yaw_degrees=None,physical_rotation_verified=False,
        review_limitation='Multiple small-angle homography poses; exact physical yaw not certified. Removal frames excluded.')
    review_path.write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8')
    candidate=json.loads(CANDIDATE.read_bytes())
    record=dict(status='bounded_contact_refinement_and_small_yaw_ready_for_operator_trial',
        date_client='2026-10-24',previous_physical_source=PHYSICAL.name,previous_physical_source_sha256=sha,
        previous_turn_commands_complete=True,previous_stop_phase='remove_cube',
        previous_all_torque_off_verified=True,previous_settings_restored=True,
        previous_goal_yaw_verified=False,cube_image_translation_px=review['cube_image_translation_px'],
        candidate=str(CANDIDATE.relative_to(ROOT)),candidate_sha256=CANDIDATE_SHA,
        refinement_samples=20,turn_samples=70,simulated_yaw_range_deg=candidate['evaluated_simulated_yaw_range_deg'],
        simulated_lateral_range_mm=candidate['evaluated_simulated_translation_range_mm'],
        same_predicted_direction_in_nine_cases=True,full_held_out_angle_criterion_passed=False,
        robust_policy=False,live_sac=False,physical_rotation_verified=False,
        native_proofs=proofs,regression_tests=regression['tests_run'],
        maximum_native_tracking_rad=max(v['maximum_tracking_rad'] for v in proofs),
        simulation_video='contact_point_yaw_simulation.mp4',
        next_operator_command='leap_grasped_yaw_trial.cmd',
        fresh_OFF_and_full_screen_required_at_runtime=True,source_images_modified=False,
        hardware_commands_sent=0,real_devices_opened=False,
        limitations=candidate['limitations'])
    (OUT/'contact_yaw_ready_review.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    title='## CURRENT 2026-10-24: 제거 전 영상 분석 완료 / 접촉 조정 후 작은 회전 시험 준비'
    section=f'''{title}

- 10월 23일 23:47 실행은 작은 회전 명령 90개와 이후 유지 40개, 제거 전 관찰까지 완료했다. Q는 `REMOVE CUBE` 안내 중 입력됐다. 모든 토크 OFF와 설정 및 카메라 노출 복원이 확인됐다. 전류 최대 90mA, 전압 최소 3.9V, 추종 오차 최대 약 0.07823rad이며 이 실행에는 읽기 및 목표 확인 오류가 없었다. 제거 완료 SPACE는 없으므로 현재 큐브가 손 밖에 있다고 가정하지 않는다.
- 제거 전 영상에서 큐브는 약 17.14픽셀 옆으로 이동했고 고정 손바닥의 이동은 약 0.03픽셀이다. 원래 목표였던 5도 수평 회전은 확인되지 않았다. 사진의 작은 마커와 평면 분석만으로 정확한 3차원 회전량을 확정할 수 없으며 제거 중 움직임은 성공 근거에서 제외했다. 원본 결과와 프레임은 수정하지 않았다.
- 새 후보는 기존 파지 뒤 1초간 최대 18틱의 접촉 조정, 이어 3.5초간 작은 회전, 2초 유지다. 손가락 끝 중심 대신 모델에서 큐브와 닿는 표면의 점으로 목표를 만들고 55mm, 100g 큐브의 접촉 동역학을 확인했다. 전류 450mA와 PID, 관절 범위, 뿌리 및 약지 고정, 전체 최대 64틱과 명령당 최대 2틱, 전압 및 추종과 통신 중단 조건은 유지했다. 빈손 약지 준비 순서와 파지 끝점도 유지했다.
- 마찰, 큐브 위치와 관절 응답을 바꾼 9개 모델 조건에서 회전 방향은 같았고 약 0.93~3.24도로 예측됐다. 옆 이동은 약 2.98~4.66mm다. 한 조건이 1도 미만이므로 견고한 회전 정책 기준에는 통과하지 못했다. 큐브 위치와 마찰은 아직 실측 등록된 값이 아니며 손의 응답도 가정이다. 이 결과는 제한된 실물 관찰 시험의 후보이며 목표 회전 성공이나 실물 SAC 학습 완료가 아니다.
- 기존 6개 저장 시작과 23:47 종료 OFF 자세의 전체 새 경로를 각각 67개 정적 및 3개 응답 조건으로 다시 실행해 통과했다. 최대 모델 추종 오차는 약 {record['maximum_native_tracking_rad']:.5f}rad이다. 장치 없는 회귀 검사 {regression['tests_run']}개도 통과했다. 실제 실행은 새 안정 OFF 각도와 전체 경로 검사를 다시 수행한다. 이 작업에서 실제 COM 포트나 카메라를 열지 않았다.
- **지금 사용자 작업:** 큐브를 손과 카메라 화면 밖으로 빼고 다른 제어 프로그램을 닫은 뒤 `leap_grasped_yaw_trial.cmd`를 실행한다. 첫 SPACE는 빈손 준비, `RING RELEASE CHECK`에서는 약지 끝이 분리됐는지 확인 후 SPACE, `PLACE CUBE`에서는 초록 위와 주황 카메라 쪽으로 놓고 손을 뺀 뒤 SPACE다. 이후 접촉 조정과 작은 회전은 자동이다. `REMOVE CUBE`에서 부드럽게 빼고 SPACE로 종료한다. 걸리면 Q로 OFF한 뒤 뺀다. 완료 후 했어만 알려주면 저장 결과를 직접 확인한다. 추가 사진이나 치수 측정은 필요 없다.
- 근거: `right_hand_rotation_20261024/contact_yaw_ready_review.json`, `contact_yaw_integration_validation.json`, `contact_yaw_regression_validation.json`. 시뮬레이션 영상은 `contact_point_yaw_simulation.mp4`, 직전 실물의 제거 전 영상은 `grasped_yaw_trial_20261023_234707_886202/grasped_small_yaw_before_removal.mp4`다.

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8')
        if text.startswith(title):
            boundary=text.find('## PREVIOUS ')
            text=text[boundary:] if boundary>=0 else ''
        else:text=text.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(section+text,encoding='utf-8')
    print(json.dumps({k:record[k] for k in ('status','regression_tests','maximum_native_tracking_rad',
        'simulated_yaw_range_deg','hardware_commands_sent','real_devices_opened')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
