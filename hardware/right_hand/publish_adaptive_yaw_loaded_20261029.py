"""Publish only after complete loaded, current recovery and source checks."""
import datetime,hashlib,json
from pathlib import Path
import numpy as np
from adaptive_yaw_loaded_plan_20261029 import AdaptiveYawLoadedPlan,ROOT,OUT,REQUEST,PROOF,EMPTY_EVIDENCE
from adaptive_yaw_loaded_gate_20261029 import validate_review
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
proof=validate_review(AdaptiveYawLoadedPlan(REQUEST),PROOF);actual=read(EMPTY_EVIDENCE)
fresh_folders=sorted(ROOT.glob('fast_off_readonly_20261029_*/result.json'));fresh_path=fresh_folders[-1];fresh=read(fresh_path)
assert fresh['servo_writes']==0 and fresh['torque_changes']==0
live_off=fresh['status']=='off_snapshots_saved' and fresh.get('maximum_drift_ticks',99)<=2
delta=int(np.max(abs(np.asarray([s['position'] for s in fresh['snapshots']])-AdaptiveYawLoadedPlan(REQUEST).source_position()))) if live_off else None
live_ready=live_off and delta<=4
pending_reason=None if live_ready else (fresh.get('error') or 'Current posture needs a new recovery screen')
for filename,digest in {
    'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
    'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
    'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
    'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d',
    'horizontal_quarter_motion_runner_20261027.py':'5c8487f134264f5ef4eb9f6bdf6a465e5fb6c69945c4a91407e1b6d75cc760cb',
    'C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py':'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'}.items():assert sha(ROOT/filename)==digest,filename
coarse_path=OUT/'loaded_pause_stress_27/review.json';coarse=read(coarse_path)
assert len(coarse['cases'])==27 and not coarse['all_passed']
coarse_summary=dict(path=coarse_path.relative_to(ROOT).as_posix(),sha256=sha(coarse_path),cases=27,
    passed=sum(c['passed'] for c in coarse['cases']),failed=sum(not c['passed'] for c in coarse['cases']),
    acceptance_criteria_not_changed=True,not_used_as_approval=True)
now=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat()
ready=dict(status='LOADED_original638_bounded_catch_up_ready' if live_ready else 'LOADED_models_complete_live_confirmation_pending',timestamp=now,proof=PROOF.relative_to(ROOT).as_posix(),proof_sha256=sha(PROOF),
    latest_readonly=fresh_path.relative_to(ROOT).as_posix(),current_pose_difference_ticks=delta,
    latest_EMPTY_evidence=EMPTY_EVIDENCE.relative_to(ROOT).as_posix(),actual_EMPTY_wait_events=actual['root_wait_events'],
    actual_EMPTY_complete=True,actual_EMPTY_ID12_yaw_max_tracking_rad=actual['ID12_yaw_max_tracking_rad'],
    actual_EMPTY_current_peak_ma=actual['current_peak_ma'],improvement_not_attributed_to_inactive_wait=True,
    actual_wait_controller_activation_verified=False,actual_loaded_wait_effect_unverified=True,
    current_recovery_static_cases=201,current_recovery_dynamic_cases=9,loaded_pause_dynamic_cases=27,
    numerical_step_refinement_cases=4,coarse_grid=coarse_summary,refined_loaded_model=proof['loaded_fixed_goal_pause_stress'],
    software_checks_passed=14,software_check_sources={f:sha(ROOT/f) for f in ('test_root_lag_pause_20261029.py','test_adaptive_yaw_loaded_20261029.py')},
    same_original_638_yaw_targets=True,angular_targets_current_and_PID_unchanged=True,
    native_sample_period_s=.075,nominal_motion_seconds=proof['nominal_motion_seconds'],active_motion_limit_s=300,
    root_lag_pause_policy=proof['root_lag_pause_policy'],force_and_health_stops_unchanged=True,
    placement_orientation='GREEN up; ORANGE toward wrist ID41',physical_rotation_unverified=True,
    cube_starts_OUT=True,cube_only_after_PLACE=True,automatic_OPEN_before_REMOVE=True,
    terminal_SPACE_checkpoints=3,assistant_motor_writes=0,protected_sources_unchanged=True,
    code_and_model_ready=True,current_pose_live_verified=live_ready,live_confirmation_pending_reason=pending_reason)
(OUT/'readiness.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
guide='''큐브를 잡고 전체 수평 회전 경로를 확인하는 시험

1. 처음에는 큐브를 손과 카메라 시야 밖에 둡니다. 손가락과 받침은 그대로 두세요.
2. leap_adaptive_yaw_loaded.cmd를 실행하고 터미널에서 첫 SPACE를 누릅니다.
3. PLACE CUBE가 뜬 뒤에만 큐브를 손바닥 위 손가락 뿌리 쪽, 엄지와 중지 사이에 놓습니다.
   초록 중앙ID5가 위, 주황은 손목ID41 쪽입니다. 카메라 쪽이라는 뜻이 아닙니다.
   이전 배치 사진: ../../calibrated_quarter_trial_20261028_174723_390729/cube_before_grasp.jpg
4. 큐브가 스스로 놓이면 손을 떼고 터미널 두 번째 SPACE를 누릅니다.
5. 잡기, 이전638개 수평 회전 목표, 고정, 자동 OPEN까지 손을 대지 말고 기다립니다.
6. REMOVE가 뜨면 큐브를 빼고 터미널 세 번째 SPACE를 눌러 OFF합니다.

실제 동작은 약3분40초이며, 중지 대기가 생기면 최대20초 추가됩니다. 배치와 제거 시간은 별도입니다.
Q/Esc/Ctrl+C로 중단합니다. 중단되면 자동으로 재시도하지 말고 했어라고 알려주세요.
전체 로그를 붙일 필요는 없습니다.

직전 빈손 시험은 회전320개와 최종 고정/OFF/원래 설정 복구까지 완료했습니다.
전체 최대 지연0.0905rad, 최대 전류114mA, 중지 회전 구간 최대 지연0.0813rad와105mA였습니다.
대기 진입0.12rad에 도달하지 않아 대기는 실제0회였고, 개선을 대기 기능의 효과로 볼 수 없습니다.

이번에는 이전 실물 완료 기록의638개 회전 목표를 그대로 사용합니다. 목표 각도/간격/전류/PID는 바꾸지 않았습니다.
회전 중 중지ID12가0.12rad 뒤처지면 새 목표 쓰기를 보류합니다. 정상 상태0.10rad 이내2회 후 진행합니다.
대기당1.2초/총20초이며 원래300초 제한에 포함되고, 따라오지 못하면 OFF합니다.
전압/온도/전류/0.18rad 추종/진행/고정 관절 검사와 목표 재전송 없음은 그대로입니다.

추가 대기를 넣은2ms 모델은27개 중 일부가 큐브 접촉1mm 기준을 넘어서 승인에 쓰지 않았습니다.
기준과 물리 조건을 유지한 채 계산 간격만1ms로 줄여27조건을 전부 확인했고,
문제가 났던 두 조건은1ms와0.5ms에서 각각 다시 확인했습니다.
접촉 깊이가 완전히 수치적으로 수렴했다고 단정하지 않으며, 모든 가능한 대기 시점이 검증된 것은 아닙니다.
모델의 회전량은 실제90도 완료의 증명이 아닙니다. 실제 회전량은 저장 영상에서 확인합니다.
'''
if not live_ready:guide='현재 USB/토크 OFF/관절값 확인이 필요합니다. 아직 동작 프로그램을 실행하지 마세요.\n이유: '+str(pending_reason)+'\n\n준비된 실행 절차:\n'+guide
(OUT/'실행안내.md').write_text(guide,encoding='utf-8')
command='& "'+str(ROOT/'leap_adaptive_yaw_loaded.cmd')+'"'
for filename in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/filename;r=read(path)
    r.update(status=ready['status'],current_status=ready['status'],timestamp=now,next_command=command,next_user_command=command,
        next_user_command_ready=live_ready,next_physical_command_ready=live_ready,next_user_action_required=True,
        next_user_action=('EMPTY first SPACE; PLACE GREEN up/ORANGE wristward; second SPACE starts full original638 yaw; automatic OPEN; REMOVE then third SPACE=OFF') if live_ready else 'Connect robot USB and5V power; cube OUT; no motion program yet; fresh READ-only confirmation required',
        next_physical_command_kind='LOADED_ORIGINAL638_WITH_BOUNDED_CATCH_UP',next_step='Review actual cube yaw and bounded wait behavior under load',
        trajectory='EMPTY recovery -> PLACE -> grasp -> original yaw638 with bounded waits -> OPEN -> REMOVE -> OFF',
        cube_orientation_at_place=ready['placement_orientation'],next_user_orientation=ready['placement_orientation'],
        nominal_motion_seconds=ready['nominal_motion_seconds'],active_motion_limit_s=300,
        source_off_readonly=ready['latest_readonly'],latest_readonly=ready['latest_readonly'],latest_pose_difference_from_review_ticks=delta,
        proof=ready['proof'],proof_sha256=ready['proof_sha256'],software_checks_passed=14,
        latest_completed_empty_result=actual['source'],latest_empty_load_comparison=actual,next_yaw_readiness=ready,
        yaw_motion_ready=live_ready,loaded_bounded_catch_up_ready=live_ready,empty_load_comparison_ready=False,adaptive_empty_yaw_ready=False,
        current_cube_removed_confirmed_by_user=True,motors_off_verified=True if live_ready else None,original_settings_restored_now=True if live_ready else None,
        last_completed_run_torque_OFF_verified=True,last_completed_run_settings_restored=True,live_confirmation_pending_reason=pending_reason,
        physical_combined_rotation_verified=False,assistant_motor_writes=0,assistant_motor_or_setting_writes=0)
    path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
notice=(('2026-10-29 현재 단계: 빈손320구간 완료 후 큐브를 올려 기존638개 수평 회전 경로를 시험할 준비가 됐다. ' if live_ready else '2026-10-29 현재 단계: 빈손320구간과 다음638회전 경로의 코드/모델 검증을 완료했다. 현재 USB/토크OFF/관절값 확인이 필요해 동작 명령은 아직 실행하지 않는다. 이유: '+str(pending_reason)+'. ')
    +'adaptive_yaw_empty_20261029_062809_665980은 끝까지 완료/OFF/원래 설정 복구, 전체 최대0.09050rad/114mA, 중지 회전 최대0.08130rad/105mA였다. '
    '대기 조건에 도달하지 않아 실제 대기0회였으므로 지연 감소를 대기 기능의 효과라고 하지 않는다. '
    '대기를 추가한2ms 모델은27개 중'+str(coarse_summary['failed'])+'개가 기존 큐브 접촉1mm 기준을 넘어서 승인에 사용하지 않았다. '
    '기준/물리조건을 유지하며 계산을1ms로 정밀화한27조건과 해당 두 조건의1ms/0.5ms4계산을 통과했다. '
    '접촉 깊이 수렴과 모든 임의 대기 시점, 실물 대기 동작은 아직 완전히 검증된 것이 아니다. '
    '현재 복귀201정적+9동적, 소프트웨어14검사도 통과했다. '
    '다음leap_adaptive_yaw_loaded.cmd는 빈손 첫SPACE,PLACE후 초록위/주황손목ID41,두번째SPACE로 전체638목표,자동OPEN,REMOVE후큐브빼고세번째SPACE로OFF한다. '
    '목표각도/전류/PID/건강/진행/추종중단 기준을 유지한다.중지 지연시현재목표를읽기만하며 대기당1.2초/총20초,기존300초안에서제한하고 못따라오면OFF한다. '
    '현재 출발점 차이'+str(delta)+'틱, 명목'+str(ready['nominal_motion_seconds'])+'초다. 실제 회전량은 저장 영상 검토가 필요하다.\n\n')
for filename in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/filename;previous=path.read_text(encoding='utf-8')
    if notice not in previous:path.write_text(notice+previous,encoding='utf-8')
print(json.dumps(dict(status=ready['status'],current_pose_difference_ticks=delta,nominal_motion_seconds=ready['nominal_motion_seconds'],
    command=command,assistant_motor_writes=0),ensure_ascii=False))
