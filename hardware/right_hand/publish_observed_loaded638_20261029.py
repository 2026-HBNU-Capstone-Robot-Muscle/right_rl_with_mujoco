"""Publish real near-quarter-turn evidence and the next EMPTY reverse check."""
import datetime,hashlib,json
from pathlib import Path
import numpy as np
from adaptive_yaw_loaded_plan_20261029 import AdaptiveYawLoadedPlan
from reverse_yaw_empty_plan_20261029 import ReverseYawEmptyPlan,ROOT,OUT,REQUEST,PROOF
from reverse_yaw_empty_gate_20261029 import validate_review
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
F=ROOT/'adaptive_yaw_loaded_20261029_160451_416563';O=F/'offline_review';raw_hash=sha(F/'result.json');run=read(F/'result.json')
assert run['status']=='adaptive_loaded_yaw_observation_complete' and run['all_torque_off_verified'] and run['settings_restored'] and run['cube_removal_confirmed']
assert run['policy_used'] is False and not run.get('read_failures',[]) and run['camera_recording']['image_signal_blank_frames']==0
forward=AdaptiveYawLoadedPlan(F/'fresh_off_request.json')
# The native runner constructs its queue from the accepted ON hold, after the
# existing <=4-tick enable verification, rather than from the review center.
goals=[q.tolist() for phase,q in forward.plan(np.asarray(run['accepted_enable_goal'],dtype=int))]
written=[e['requested'] for e in run['write_events'] if e['address']==116];assert written[-len(goals):]==goals
angle=read(O/'marker_labelled_grid_trace.json');old_method=read(O/'grid_rotation_review.json')
a=angle['comparisons']['placement_to_turn'];b=angle['comparisons']['placement_to_open'];yaw=angle['comparisons']['grasp_to_turn']
assert a['before_frames']>=3 and a['after_frames']>=20 and b['after_frames']>=20
assert 80<a['twist_deg_p05_median_p95'][1]<100 and 75<b['twist_deg_p05_median_p95'][1]<100
assert abs(a['twist_deg_p05_median_p95'][1]-old_method['comparisons']['placement_to_turn']['twist_deg_p05_median_p95'][1])<1.
videos=read(O/'video_exports.json')
for e in videos['exports']:
    assert e['playback_speed']==1 and sha(ROOT/e['video'])==e['sha256'] and (ROOT/e['video']).stat().st_size>100000
current_request=OUT/'live_recovery_162936/request.json';current_proof=OUT/'live_recovery_162936/review.json'
proof=validate_review(ReverseYawEmptyPlan(current_request),current_proof)
fresh_path=sorted(ROOT.glob('fast_off_readonly_20261029_*/result.json'))[-1];fresh=read(fresh_path)
assert fresh['status']=='off_snapshots_saved' and fresh['maximum_drift_ticks']<=2 and not fresh['servo_writes'] and not fresh['torque_changes']
delta=int(np.max(abs(np.asarray([s['position'] for s in fresh['snapshots']])-ReverseYawEmptyPlan(current_request).source_position())))
assert delta<=4,'Current reverse start needs a new EMPTY recovery review'
protected={
    'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
    'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
    'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
    'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d',
    'horizontal_quarter_motion_runner_20261027.py':'5c8487f134264f5ef4eb9f6bdf6a465e5fb6c69945c4a91407e1b6d75cc760cb',
    'C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py':'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'}
for filename,digest in protected.items():assert sha(ROOT/filename)==digest,filename
now=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat()
physical=dict(status='observed_physical_near_quarter_turn_complete',timestamp=now,source_result=(F/'result.json').relative_to(ROOT).as_posix(),
    source_result_sha256=raw_hash,source_record_and_images_unchanged=True,actual_all_goal_writes_match_reviewed_queue=True,
    all_torque_off_verified=True,settings_restored=True,cube_removal_confirmed=True,
    physical_cube_rotation_observed=True,approximate_one_direction90_demo_observed=True,
    placed_to_held_angle_deg=a['twist_deg_p05_median_p95'][1],placed_to_open_angle_deg=b['twist_deg_p05_median_p95'][1],
    yaw_segment_only_deg=yaw['twist_deg_p05_median_p95'][1],angle_includes_grasp_rotation=True,
    top_face='GREEN ID5 retained',visible_front_change='ORANGE -> WHITE',
    angle_analysis=(O/'marker_labelled_grid_trace.json').relative_to(ROOT).as_posix(),
    cross_check=(O/'grid_rotation_review.json').relative_to(ROOT).as_posix(),
    fixed_palm_image_check=(O/'fixed_palm_image_check.json').relative_to(ROOT).as_posix(),
    exact_robot_frame90_certified=False,camera_body_motion_not_precisely_calibrated=True,
    palm_image_registration_was_not_consistently_successful=True,frame_spread_not_absolute_accuracy=True,
    visible_base_steadied_by_human=True,continuous_absence_of_human_cube_assistance_not_separately_certified=True,
    root_wait_events=len(run.get('root_lag_wait_events',[])),wait_effect_not_demonstrated=True,
    read_failures=0,minimum_voltage_v=min(min(s['voltage_v']) for s in run['samples']),
    peak_current_ma=max(max(abs(c) for c in s['current_ma']) for s in run['samples']),
    maximum_tracking_rad=max(max(s['tracking_rad']) for s in run['samples']),camera_frames=run['camera_recording']['frame_count'],
    camera_blank_frames=0,video_exports=videos['exports'],live_SAC_used=False,
    all_directions_verified=False,repeatability_across_placements_verified=False,full_cube_solve_verified=False,assistant_motor_writes=0)
save(O/'physical_review.json',physical)
command='& "'+str(ROOT/'leap_reverse_yaw_empty.cmd')+'"'
ready=dict(status='EMPTY_reverse_path_ready_after_observed_quarter_turn',timestamp=now,
    proof=current_proof.relative_to(ROOT).as_posix(),proof_sha256=sha(current_proof),source_request=current_request.relative_to(ROOT).as_posix(),latest_readonly=fresh_path.relative_to(ROOT).as_posix(),
    current_pose_difference_ticks=delta,current_recovery_static_cases=201,current_recovery_dynamic_cases=9,
    reverse_hand_dynamics_cases=9,software_checks_passed=7,test_source_sha256=sha(ROOT/'test_reverse_yaw_empty_20261029.py'),
    nominal_motion_seconds=proof['nominal_motion_seconds'],active_motion_limit_s=300,
    cube_stays_OUT=True,one_terminal_SPACE_only=True,no_PLACE_or_REMOVE=True,
    inverse_cube_trial_ready=False,physical_inverse_rotation_verified=False,assistant_motor_writes=0,
    next_user_command=command,latest_physical_review=(O/'physical_review.json').relative_to(ROOT).as_posix())
save(OUT/'readiness.json',ready)
guide='''반대방향 경로의 빈손 확인

큐브는 처음부터 끝까지 손과 카메라 시야 밖에 둡니다. 손가락과 받침은 그대로 두세요.
leap_reverse_yaw_empty.cmd를 실행하고 터미널 SPACE 한 번만 누릅니다.
현재 자세에서 준비 자세로 이동한 뒤, 이번에 도달한 관절 경로를 역순으로 움직이고 자동 OFF합니다.
약3~4분이며 PLACE나 REMOVE, 추가 SPACE는 없습니다. Q/Esc/Ctrl+C로 중단합니다.
완료하거나 멈추면 했어라고 알려주세요. 전체 로그를 붙일 필요는 없습니다.

이번 실물 실행은 배치→잡은 상태 약88.7도, 손을 편 뒤 약83.0도 회전을 영상에서 관측했습니다.
회전 지령 구간만의 기여는 약70.9도이며 전체 값에는 잡는 과정의 회전도 포함합니다.
정확한 로봇 좌표90도나 모든 방향의 제어, 배치 변화에 대한 반복성은 아직 인증한 것이 아닙니다.
모터 OFF/설정 복구/큐브 제거를 확인했고, 상태 수신 오류0건과 유효 카메라2107장을 확보했습니다.
대기 동작은0회였으므로 개선을 대기 기능의 효과라고 하지 않습니다.

다음 빈손 경로는 기존 각도를 늘리지 않고 실제 도달한 목표를 역순으로 사용합니다.
현재 복귀201정적+9동적, 역방향 손 동역학9조건, 소프트웨어7검사를 통과했습니다.
전류/PID/전압/온도/추종/진행 검사와75ms 간격은 그대로입니다. 아직 큐브를 올리지 않습니다.
'''
(OUT/'실행안내.md').write_text(guide,encoding='utf-8')
for filename in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/filename;r=read(path)
    r.update(status=ready['status'],current_status=ready['status'],timestamp=now,next_command=command,next_user_command=command,
        next_user_command_ready=True,next_physical_command_ready=True,next_user_action_required=True,
        next_user_action='EMPTY throughout; one terminal SPACE; reverse reached joint cycle; automatic OFF; no cube placement',
        next_physical_command_kind='EMPTY_REVERSE_JOINT_CYCLE',next_step='Verify the opposite joint direction EMPTY before developing loaded inverse rotation',
        trajectory='EMPTY preparation -> reverse joint cycle -> hold -> OFF',cube_orientation_at_place=None,next_user_orientation=None,
        nominal_motion_seconds=ready['nominal_motion_seconds'],active_motion_limit_s=300,
        source_off_readonly=ready['latest_readonly'],latest_readonly=ready['latest_readonly'],latest_pose_difference_from_review_ticks=delta,
        proof=ready['proof'],proof_sha256=ready['proof_sha256'],software_checks_passed=7,next_yaw_readiness=ready,
        latest_physical_review=ready['latest_physical_review'],latest_completed_loaded_result=physical['source_result'],
        approximate_one_direction90_demo_observed=True,physical_cube_rotation_observed=True,
        placed_to_held_angle_deg=physical['placed_to_held_angle_deg'],placed_to_open_angle_deg=physical['placed_to_open_angle_deg'],
        physical_horizontal_90_verified=False,legacy_90_flag_scope='Exact calibrated robot-frame90 is not certified; approximate camera-observed quarter-turn demo is separately recorded',
        physical_combined_rotation_verified=False,inverse_cube_rotation_verified=False,
        yaw_motion_ready=False,loaded_bounded_catch_up_ready=False,empty_load_comparison_ready=False,adaptive_empty_yaw_ready=False,
        reverse_empty_yaw_ready=True,current_cube_removed_confirmed_by_user=True,motors_off_verified=True,original_settings_restored_now=True,
        assistant_motor_writes=0,assistant_motor_or_setting_writes=0)
    save(path,r)
notice=('2026-10-29 16:04 실물 회전 완료: adaptive_yaw_loaded_20261029_160451_416563은 잡기,638개 회전 목표, 고정,자동OPEN,큐브제거,OFF와설정복구까지 완료했다. '
    '배치→잡고있는상태는같은초록ID5로약88.7도,손을편뒤약83.0도다.회전구간만약70.9도이고잡는과정의회전도전체에포함한다. '
    '초록윗면을유지하며보이는앞면이주황에서흰색으로바뀌었다.정밀로봇좌표각도나모든배치반복성/모든방향/큐브맞추기완료를뜻하지않는다. '
    '본체를사람손으로받치는모습이있으며팔과본체움직임을정밀보정한측정은아니다. '
    '유효카메라2107장,검정0장,통신실패0건,최저4.7V,최대98mA,최대추종0.07977rad였다.대기0회이므로대기기능의효과로해석하지않는다. '
    '원속도전체영상271.5초와잡기/회전/OPEN영상157.3초를offline_review에저장했다. '
    '다음은leap_reverse_yaw_empty.cmd로반대방향관절경로빈손확인이다.실제로도달한경로를역순으로사용하며새각도/힘/건강기준을추가하지않았다. '
    '현재복귀201정적+9동적과역방향9동적,소프트웨어7검사통과.큐브는끝까지밖,터미널SPACE1회,약3~4분,자동OFF다. '
    '근거: '+ready['latest_physical_review']+'\n\n')
for filename in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/filename;old=path.read_text(encoding='utf-8')
    if notice not in old:path.write_text(notice+old,encoding='utf-8')
assert sha(F/'result.json')==raw_hash
print(json.dumps(dict(status=ready['status'],held_angle_deg=physical['placed_to_held_angle_deg'],open_angle_deg=physical['placed_to_open_angle_deg'],
    current_pose_difference_ticks=delta,nominal_motion_seconds=ready['nominal_motion_seconds'],command=command,assistant_motor_writes=0),ensure_ascii=False))
