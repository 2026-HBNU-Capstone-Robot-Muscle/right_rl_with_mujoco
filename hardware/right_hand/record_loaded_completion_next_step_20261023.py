"""Save the actual result and the single missing physical observation."""
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'cube_supported_rotation_trial_20261023_034640_750784'
review=json.loads((folder/'completion_review.json').read_bytes())
trial=json.loads((folder/'result.json').read_bytes())
load=json.loads((folder/'empty_vs_loaded_grasp.json').read_bytes())
screen=json.loads((folder/'conditional_grip_closure_screen.json').read_bytes())
proof=json.loads((folder/'current_start_screen.json').read_bytes())
hashes=proof['extra_planner_sha256']
capture=ROOT/'grasp_place_capture.py'
expected=hashes['grasp_place_capture.py']
blob=capture.read_bytes()
if hashlib.sha256(blob).hexdigest()!=expected:
    # Restore original line endings only if the entire original evidence hash
    # matches. Never change the saved proof to accommodate new code.
    options=[blob.replace(b'\r\n',b'\n'),blob.replace(b'\r\n',b'\n').replace(b'\n',b'\r\n')]
    restored=next((v for v in options if hashlib.sha256(v).hexdigest()==expected),None)
    if restored is None:raise RuntimeError('Capture differs from the completed-run controller')
    capture.write_bytes(restored)
unchanged={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==sha for name,sha in hashes.items()}
assert all(unchanged.values()),[k for k,v in unchanged.items() if not v]
out=dict(status='loaded_path_complete_contact_observation_needed',source=folder.name,
    source_result_sha256=review['source_result_sha256'],recorded_rotation_targets=256,
    exact_previous_targets=True,final_target_exact=True,
    all_torque_off_verified=True,settings_restored=True,
    min_voltage_v=review['min_voltage_v'],peak_current_ma=review['peak_current_ma'],max_tracking_rad=review['max_tracking_rad'],
    valid_camera_frames=trial['camera_recording']['image_signal_usable_frames'],
    visible_desired_rotation_verified=False,
    video_observation='Front/top faces and edges change little during the isolated rotation interval.',
    matched_grasp_samples=load['goal_samples_identical'],
    max_absolute_median_added_signed_current_ma=max(abs(r['added_signed_current_ma']) for r in load['rows']),
    contact_force_calibrated=False,contact_geometry_unresolved=True,
    closure_proposals_exported=False,
    all_controller_and_evidence_fingerprints_unchanged=True,
    next_launcher='leap_cube_contact_view.cmd',
    next_protocol='Existing v6 grasp only; at Approach ended/REMOVE prompt, move camera to opposite-thumb side and show INDEX/MIDDLE tip-to-cube gaps for 5-10s. Then remove cube and SPACE for OFF.',
    camera_marker_gate=False,agent_hardware_commands_sent=0,
    motor_targets_or_limits_changed=False)
(ROOT/'right_hand_rotation_20261023/loaded_completion_contact_review.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
text='''## CURRENT 2026-10-23 03:46 이후: 큐브 포함 경로 완주, 접촉 측면 관측 필요

- `cube_supported_rotation_trial_20261023_034640_750784`은 회전 목표256개와 마지막 목표까지 정확히 실행했다. 워치독 정지는 재발하지 않았으며 토크OFF·설정 복원을 확인했다. 최저3.9V, 최대98mA, 최대추종0.08590rad. 정상1872프레임·빈영상0개다.
- 쥐기 동작과 분리한 마지막 회전 구간 약13초를 검토했다. 앞면/윗면·모서리의 변화가 작아 원하는 실물 회전 성공으로 기록하지 않는다. 원본110프레임은 실행 폴더의 `recorded_loaded_rotation.mp4`로 보존했다. 작은 윗면 마커의2D 모서리 변화약-2도는 중력 기준 회전각이 아니다.
- 기존 빈손 완주의 동일 쥐기목표322개와 비교했다. 마지막30표본의 추가전류 중앙값은 관절별 절대최대8mA다. 힘을 교정한 측정이나 무접촉 증명은 아니지만 파지 부하가 크게 늘었다는 근거도 없다.
- 같은 실행의 제거 대기 영상에서ID40/41 동시 프레임5개를 확보했다. 작은 단일 마커의 위치 추정과55mm 전체테두리 제약 추정은 약5cm 차이를 보였다. 모델/카메라 등록은 미확정이며, 이 차이를 실물 위치오차로 확정하지 않는다. 테두리의 수동 코너에는±3px 불확실성을 명시했다. 과거 근접 촬영의 마커 형상도 별도 진단으로 비교했지만 보정값으로 설치하지 않았다.
- 작은 파지변경3개와 무변경을 정적 모델에서 비교했다. 손 자체/받침 겹침은 없었지만 미확정 큐브 위치27개에서 양쪽 손끝 접촉을 확보한 안은 없었다. 정적 조건은 실제 힘/동역학과 다르므로 파지불가능의 증명도 아니다. 이 안들을 실물 목표로 내보내거나 전류·추종 기준을 올리지 않았다.
- 다음 사용자 작업: `leap_cube_contact_view.cmd`. 기존v6 쥐기 경로를 그대로 사용하며 추가 회전/더 강한 파지는 없다. 큐브 없이 시작→기존 약지분리 확인→PLACE때 배치/SPACE. **Approach ended / REMOVE CUBE**에서 바로 빼지 말고 카메라를 엄지 반대쪽 옆면으로 옮겨 검지·중지 끝과 큐브 사이를5~10초 보여준다. 이후 큐브를 빼고SPACE로OFF. 원래90초 제거대기 제한/Q중단을 유지한다. 마커는 필요 없다. 영상은자동저장하므로 했어만 알려주면 된다.
- 실행 경로/전류/속도/상태검사/OFF코드는 완주 때와 모든 증거파일 해시가 일치한다. 새cmd는촬영 안내 후 기존cmd를호출하는진입점이다. 조수는 실제 모터/카메라를열지 않았다. 근거: `right_hand_rotation_20261023/loaded_completion_contact_review.json` 및 실행폴더의 `completion_review.json`, `empty_vs_loaded_grasp.json`, `conditional_grip_closure_screen.json`.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8')
    if not old.startswith('## CURRENT 2026-10-23 03:46 이후:'):p.write_text(text+old,encoding='utf-8')
print(json.dumps(out,indent=2))
