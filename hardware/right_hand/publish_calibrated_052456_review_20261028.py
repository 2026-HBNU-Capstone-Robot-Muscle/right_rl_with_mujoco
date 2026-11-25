"""Publish saved-run near-quarter observation, without changing raw reports."""
import datetime as dt,hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
run=ROOT/'calibrated_quarter_trial_20261028_052456_066430'
out=ROOT/'right_hand_rotation_20261028/completed_calibrated_052456'
r=json.loads((run/'result.json').read_bytes());measured=json.loads((out/'same_marker_rotation_review.json').read_bytes());trace=json.loads((out/'rotation_trace.json').read_bytes())
samples=r['samples'];frames=json.loads((run/'camera/frames.json').read_bytes())['frames']
def phase_stats(phase):
    rows=[v for v in trace['rows'] if v['accepted'] and v['phase']==phase]
    return dict(accepted_frames=len(rows),observed_span_s=rows[-1]['time_after_enabled_s']-rows[0]['time_after_enabled_s'] if rows else None,
        camera_relative_yaw_deg_min_median_max=np.percentile([v['angle_deg'] for v in rows],[0,50,100]).tolist() if rows else None)
comparisons=measured['comparisons'];held=comparisons['placement_to_turn']['twist_total_tilt_p05_median_p95'][1][0];opened=comparisons['placement_to_open']['twist_total_tilt_p05_median_p95'][1][0]
review=dict(status='near_quarter_turn_while_grasped_with_release_unwind',source_run=run.relative_to(ROOT).as_posix(),
    source_result_sha256=hashlib.sha256((run/'result.json').read_bytes()).hexdigest(),
    command_sequence_complete=r['status']=='calibrated_quarter_observation_complete',
    all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],cube_removal_confirmed=r['cube_removal_confirmed'],
    read_failures=len(r.get('read_failures',[])),camera_frames=len(frames),black_or_unusable_frames=sum(v.get('image_signal_usable') is not True for v in frames),
    min_voltage_v=min(min(s['voltage_v']) for s in samples),peak_abs_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
    max_tracking_rad=max(max(s['tracking_rad']) for s in samples),max_temperature_c=max(max(s['temperature_c']) for s in samples),
    source_marker='same green center ID5',angle_reference='placed cube before loaded grasp',
    held_camera_relative_yaw_estimate_deg=held,after_open_camera_relative_yaw_estimate_deg=opened,
    apparent_release_unwind_deg=held-opened,hold_trace=phase_stats('stage_8_horizontal_hold'),after_open_trace=phase_stats('stage_10_remove_hold'),
    camera_relative_near90_while_grasped_observed=True,exact_robot_frame90_certified=False,
    final_released90_verified=False,repeated_same_route_near90_verified=False,
    single_marker_planar_ambiguity_present=True,measurement_percentiles_are_not_absolute_accuracy=True,
    static_palm_feature_check=measured['static_palm_feature_check'],
    physical_horizontal_rotation_observed=True,full_cube_solving_or_other_rotation_axes_verified=False,
    only_same638_goal_yaw_route_is_under_review=True,no_new_physical_trial_started_by_assistant=True,
    original_result_modified=False,assistant_motor_writes=0,
    video_preview_4x=(out/'cube_rotation_preview_4x.mp4').relative_to(ROOT).as_posix(),
    video_realtime=(out/'full_run_realtime.mp4').relative_to(ROOT).as_posix(),
    timestamp=dt.datetime.now().astimezone().isoformat())
(out/'physical_review.json').write_bytes((json.dumps(review,indent=2)+'\n').encode())
text='2026-10-28 05:24 단축 회전 실물 관찰: 638개 회전 목표를 사용하는 보정판이 준비, 파지, 회전, 고정, 자동 OPEN과 큐브 제거까지 완주했다. 같은 초록 ID5의 카메라 기준 회전 추정은 파지 고정 중 약88도, OPEN 후 약82도였다. 따라서 파지한 상태의 수평90도 근처 회전은 한 번 관찰됐으나, 손을 편 뒤의90도 유지나 반복성은 아직 확인되지 않았다. 원본 결과의 인증 필드는 고치지 않았고, 정확한 로봇 좌표계 각도로 인증하지 않는다.\n\n통신 오류0, 저장1790프레임에서 검은 화면0, 토크OFF와 설정 복구 및 큐브 제거 확인. 실물 동영상은 `right_hand_rotation_20261028/completed_calibrated_052456/cube_rotation_preview_4x.mp4`와 `full_run_realtime.mp4`에 보존했다. 다음은 회전 목표와 힘을 바꾸지 않고 같은 단축 동작을 한 번 더 실행해 파지 중 회전량이 다시90도 근처에 오는지 확인한다. 이후에 목표 자세를 잡고 유지하는 단계와 다른 회전축을 진행한다. 전체 큐브 풀기 완료로 보고하지 않는다. 현재 근거 `right_hand_rotation_20261028/completed_calibrated_052456/physical_review.json`.\n\n<!-- END CALIBRATED052456 PHYSICAL OBSERVATION 20261028 -->\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_text(encoding='utf-8-sig')
    if '<!-- END CALIBRATED052456 PHYSICAL OBSERVATION 20261028 -->' not in old:path.write_bytes((text+old).encode())
print(json.dumps({k:review[k] for k in ('status','held_camera_relative_yaw_estimate_deg','after_open_camera_relative_yaw_estimate_deg','hold_trace','read_failures','camera_frames','black_or_unusable_frames','min_voltage_v','peak_abs_current_ma','max_tracking_rad','all_torque_off_verified','settings_restored')}))
