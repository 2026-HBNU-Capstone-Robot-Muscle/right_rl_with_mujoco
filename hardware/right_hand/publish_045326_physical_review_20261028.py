"""Publish honest saved-run outcome and measured calibration point."""
import datetime as dt,hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
run=ROOT/'horizontal_quarter_trial_20261028_045326_636152'
out=ROOT/'right_hand_rotation_20261028/completed_quarter_045326'
r=json.loads((run/'result.json').read_bytes());trace=json.loads((out/'rotation_trace.json').read_bytes())
samples=r['samples'];frames=json.loads((run/'camera/frames.json').read_bytes())['frames']
review=dict(status='full_motion_completed_but90_degree_target_overshot',source_run=run.relative_to(ROOT).as_posix(),
    source_result_sha256=hashlib.sha256((run/'result.json').read_bytes()).hexdigest(),
    command_sequence_complete=r['status']=='horizontal_quarter_observation_complete',
    all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],cube_removal_confirmed=r['cube_removal_confirmed'],
    camera_frames=len(frames),read_failures=len(r.get('read_failures',[])),camera_image_signal=r.get('camera_image_signal'),
    min_voltage_v=min(min(s['voltage_v']) for s in samples),peak_abs_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
    max_tracking_rad=max(max(s['tracking_rad']) for s in samples),max_temperature_c=max(max(s['temperature_c']) for s in samples),
    final_camera_relative_yaw_estimate_deg=128.,physical_rotation_observed=True,
    physical90_degree_completion_verified=False,repeated90_degree_reliability_verified=False,
    angle_is_approximate_not_certified_robot_frame=True,planar_pose_ambiguity_present_in_unfiltered_pnp=True,
    source_marker='same green center ID5',observed90_degree_point=trace['closest_observed_targets']['90'],
    first_012305_observation_approx89_degrees_preserved=True,
    two_runs_are_not_two_successful90_degree_trials=True,
    video_preview_4x=(out/'cube_rotation_preview_4x.mp4').relative_to(ROOT).as_posix(),
    video_realtime=(out/'full_run_realtime.mp4').relative_to(ROOT).as_posix(),
    camera_or_communication_permanent_resolution_not_proven=True,assistant_motor_writes=0,
    timestamp=dt.datetime.now().astimezone().isoformat())
(out/'physical_review.json').write_bytes((json.dumps(review,indent=2)+'\n').encode())
text='2026-10-28 04:53 실물 실행 검토: 준비, 파지, 회전, 자동 OPEN과 큐브 제거까지 완주했고, 토크 OFF와 설정 복구를 확인했다. 통신 오류0, 저장 카메라2378프레임에서 검은 화면0이었다. 그러나 같은 초록 ID5 마커의 전후 비교는 카메라 기준 약128도이며 목표90도를 지나쳤다. 앞선01:23 실행의 약89도 관찰과 합쳐 두 번의90도 성공으로 계산하지 않는다. 모델과 실제 회전량에 차이가 있고, 현재 방식은 실시간 비전 피드백이나 라이브 SAC가 아니다.\n\n저장 영상에서 약90도를 통과한638번째 회전 목표를 찾았고, 해당 프레임의 명령16개와 원본 경로의638번째 명령이 정확히 일치했다. 원래1302개 중 이 지점까지의 목표를 그대로 사용하는 별도 보정 후보를 만들었다. 파지와 전류, PID 및 건강 중단 기준은 유지하고, 이 중간 지점에서 고정 후 자동 OPEN하는 새 부분은 별도 시뮬레이션 검토 중이다. 아직 단축 후보의 실물90도는 확인되지 않았다. 기존 명령을 반복하지 말고, 새 검토와 실행 명령이 준비될 때까지 손과 연결을 그대로 둔다. 현재 근거 `right_hand_rotation_20261028/completed_quarter_045326/physical_review.json`.\n\n<!-- END OVERSHOOT OBSERVATION 045326 20261028 -->\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8-sig');marker='<!-- END OVERSHOOT OBSERVATION 045326 20261028 -->'
    if marker not in old:p.write_bytes((text+old).encode('utf-8'))
print(json.dumps({k:review[k] for k in ('status','camera_frames','read_failures','min_voltage_v','peak_abs_current_ma','max_tracking_rad','max_temperature_c','all_torque_off_verified','settings_restored')}))
