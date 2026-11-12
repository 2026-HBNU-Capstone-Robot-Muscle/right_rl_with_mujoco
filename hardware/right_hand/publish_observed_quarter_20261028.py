"""Publish offline evidence without modifying the original physical run."""
import datetime as dt
import hashlib
import json
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,HorizontalQuarterPlan,hashes
from horizontal_quarter_gate_20261027 import validate_review

def read(path):return json.loads(path.read_bytes())
def save(path,value):path.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode('utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def rel(path):return path.relative_to(ROOT).as_posix()

proof=validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
verification=read(OUT/'verification.json')
assert verification['tests_passed']==122 and verification['code_hashes']==hashes()
run=ROOT/'horizontal_quarter_trial_20261028_012305_894520'
dest=ROOT/'right_hand_rotation_20261028/completed_quarter_012305'
r=read(run/'result.json');raw_hash=sha(run/'result.json')
assert r['status']=='horizontal_quarter_observation_complete'
assert r['all_torque_off_verified'] and r['settings_restored'] and r['cube_removal_confirmed']
assert r['policy_used'] is False
assert r['camera_recording']['image_signal_blank_frames']==0
angle=read(dest/'same_marker_rotation_review.json')
estimate=angle['comparisons']['placement_to_open']['twist_total_tilt_p05_median_p95']
assert 80<estimate[1][0]<100
videos=read(dest/'video_exports.json')
for export in videos['exports']:
    assert (ROOT/export['video']).stat().st_size>100000
    assert export['probe']['streams'][0]['codec_name']=='h264'
    export['sha256']=sha(ROOT/export['video'])

grid=dest/'grid_rotation_review.json'
if grid.exists():
    rejected=read(grid)
    rejected.update(status='rejected_incomplete_green_grid_fit',used_for_angle_claim=False,
        rejection_reason='No complete accepted placement/turn comparison; retain failed diagnostic, do not loosen fit thresholds.')
    save(grid,rejected)

samples=r['samples']
report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),
    status='single_physical_horizontal_quarter_observed_repeat_ready',
    source_result=rel(run/'result.json'),source_result_sha256=raw_hash,
    original_run_modified=False,command_sequence_completed=True,
    all_torque_off_verified=True,settings_restored=True,cube_removal_confirmed=True,
    single_trial_horizontal_quarter_observed=True,observed_trial_count=1,
    rotation_relative_to='placed cube before grasp -> end of automatic OPEN, before removal',
    rotation_degrees_estimate=estimate[1][0],sample_spread_p05_p95_degrees=[estimate[0][0],estimate[2][0]],
    sample_spread_is_absolute_accuracy_or_confidence_interval=False,
    green_top_normal_change_degrees_estimate=estimate[1][2],
    visible_face_change=dict(top='green ID5 remains up',front_before='orange ID3',front_after='white ID0'),
    angle_method=rel(dest/'same_marker_rotation_review.json'),
    static_palm_feature_displacement_px_median=angle['static_palm_feature_check']['shift_px_median_p95'][0],
    angle_limitations=['Single planar marker with saved camera calibration; not a certified robot/world-frame measurement.',
        'Intermediate turn-hold IPPE poses include ambiguous outliers; final placement-to-OPEN comparison is the reported estimate.',
        'Overall rotation includes grasp and OPEN; the yaw segment alone must not be called a measured 90-degree rotation.'],
    exact_robot_frame_angle_verified=False,repeatability_verified=False,
    arbitrary_direction_control_verified=False,left_hand_integration_verified=False,
    live_SAC_used=False,continuous_no_human_cube_assistance_separately_certified=False,
    visual_observation='Inspected endpoint images show the base held; no visible direct cube assistance in those inspected images.',
    measured_min_voltage_v=min(v for sample in samples for v in sample['voltage_v']),
    measured_peak_current_ma=max(abs(v) for sample in samples for v in sample['current_ma']),
    measured_max_tracking_rad=max(v for sample in samples for v in sample['tracking_rad']),
    camera_frames=r['camera_recording']['frame_count'],camera_blank_frames=0,
    read_failures=len(r['read_failures']),read_failures_recovered=sum(bool(f.get('recovered')) for f in r['read_failures']),
    video_exports=videos['exports'],current_off_reference='repeat_off_readonly_20261028_014813_732889/result.json',
    repeat_source=rel(REQUEST),repeat_model_proof=rel(PROOF),repeat_model_conditions=210,
    device_free_tests_passed=122,working_controller_and_loaded_goals_preserved=True,
    assistant_motor_writes=0,next_user_command='leap_horizontal_quarter_trial.cmd')
save(dest/'physical_review.json',report)

archive=dest/'before_publication';archive.mkdir(exist_ok=True)
def archive_json(path):
    target=archive/(path.parent.name+'__'+path.name)
    if path.exists() and not target.exists():target.write_bytes(path.read_bytes())

ready_path=OUT/'readiness.json';archive_json(ready_path);ready=read(ready_path)
ready.update(status=report['status'],timestamp=report['timestamp'],physical_motion_after_reseating_verified=True,
    single_trial_horizontal_quarter_observed=True,observed_trial_count=1,
    physical_horizontal_90_verified=False,
    physical_horizontal_90_flag_scope='Exact calibrated robot-frame angle is not certified; approximate single-trial quarter-turn is observed in the separate offline review.',
    physical_review=rel(dest/'physical_review.json'),rotation_degrees_estimate=report['rotation_degrees_estimate'],
    repeatability_verified=False,underlying_fault_proven_resolved=False,
    latest_readonly=report['current_off_reference'],current_source=rel(REQUEST),operator_proof=rel(PROOF),
    current_empty_static_cases=201,current_empty_dynamic_cases=9,next_user_command_ready=True,
    pending_user_input=None,next_user_task='Repeat the same sequence once. Leave fingers unchanged, close Wizard, EMPTY first SPACE. At PLACE: green up, orange toward wrist ID41; remove human hand then second SPACE. After automatic OPEN remove cube, third SPACE for OFF. Send done only.')
save(ready_path,ready)

for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
             'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name;archive_json(path);value=read(path)
    value.update(status=report['status'],current_status=report['status'],timestamp=report['timestamp'],
        latest_physical_review=rel(dest/'physical_review.json'),single_trial_horizontal_quarter_observed=True,
        rotation_degrees_estimate=report['rotation_degrees_estimate'],observed_trial_count=1,
        horizontal_quarter_turn_verified=False,
        legacy_verification_flag_scope='Exact robot-frame angle/repeatability remain uncertified; see single_trial_horizontal_quarter_observed and latest_physical_review.',
        repeatability_verified=False,arbitrary_direction_rotation_verified=False,
        latest_readonly=report['current_off_reference'],current_hand_review=rel(ready_path),
        horizontal_loaded_execution_ready=True,next_user_command='leap_horizontal_quarter_trial.cmd',
        next_user_command_ready=True,pending_user_input=None,
        next_user_action=ready['next_user_task'])
    save(path,value)

marker='<!-- END FIRST OBSERVED HORIZONTAL QUARTER 20261028 -->'
header='''2026-10-28 실물 수평 회전 첫 관측: 01:23 실행에서 준비, 파지, 회전, 자동 OPEN, 큐브 제거, 토크 OFF와 설정 복구까지 완료했다. 초록 ID5가 윗면을 유지하고 정면은 주황 ID3에서 흰색 ID0으로 바뀌었다. 배치 직후부터 자동 OPEN 완료까지 같은 초록 마커로 추정한 회전은 약 89.4도다. 이는 한 방향 한 번의 영상 관측이며, 정확한 로봇 좌표 각도와 반복 성공 및 임의 방향 제어는 아직 입증하지 않았다. 회전 구간만의 90도라고 해석하지 않는다. 전체 동작에는 파지 중 회전과 OPEN 후 되돌림도 포함된다.

실물 최저 전압 4.7V, 최대 전류 143mA, 최대 추종 오차 약 0.100rad. 카메라 2663장 모두 유효했고, 상태 수신 오류 한 번은 복구 후 정상 상태 두 번을 확인해 이어갔다. 종료 후 읽은 현재 자세의 복귀 모델 210조건과 제어 테스트 122개 통과. 성공한 목표 경로와 제어기는 그대로 유지했다. 다음은 같은 `leap_horizontal_quarter_trial.cmd`로 한 번 더 재현 확인이다. 손은 그대로 두고 빈손 첫 SPACE, PLACE에서 초록 위와 주황 손목 ID41 쪽으로 놓고 손을 뺀 뒤 두 번째 SPACE, 자동 OPEN 후 큐브를 빼고 세 번째 SPACE. ‘했어’만 보내면 저장 기록을 검토한다.

근거: `right_hand_rotation_20261028/completed_quarter_012305/physical_review.json`. 같은 폴더의 `cube_rotation_preview_4x.mp4`는 52.9초 확인본, `full_run_realtime.mp4`는 약 5분 43초 원속도 전체본이다. 원래 실행 기록과 원본 사진은 수정하지 않았다.

'''+marker+'\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_bytes()
    if marker.encode('utf-8') not in old:path.write_bytes(header.encode('utf-8')+old)
(OUT/'operator_guide.md').write_bytes(('''# 동일 조건 재현 확인

한 방향의 약 90도 실물 회전을 한 번 관측했다. 현재 자세의 복귀 검사와 제어 테스트를 통과했으며 성공한 동작은 변경하지 않았다.

1. 위자드를 닫고 손가락은 그대로 둔다. 큐브는 손과 카메라 밖에 둔 채 `leap_horizontal_quarter_trial.cmd`를 실행하고 첫 SPACE를 누른다.
2. PLACE 때 엄지와 중지 사이 손가락 뿌리 쪽에 큐브를 놓는다. 초록 위, 주황은 손목 ID41 방향이다. 사람 손을 뺀 뒤 두 번째 SPACE를 누른다.
3. 회전과 자동 OPEN이 끝나면 큐브를 빼고 세 번째 SPACE를 누른다. ‘했어’만 보내면 된다.

첫 관측과 영상은 `../completed_quarter_012305/physical_review.json`에 기록했다. 첫 관측의 최종 약 89도는 배치부터 자동 OPEN 종료까지의 추정값이다. 반복성과 다른 목표 방향은 이어서 확인한다.
''').encode('utf-8'))
assert sha(run/'result.json')==raw_hash
assert verification['code_hashes']==hashes()
validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
print(json.dumps(dict(status=report['status'],angle_estimate=report['rotation_degrees_estimate'],
    video_count=len(videos['exports']),tests=122,model_cases=210,raw_record_unchanged=True,motor_writes=0)))
