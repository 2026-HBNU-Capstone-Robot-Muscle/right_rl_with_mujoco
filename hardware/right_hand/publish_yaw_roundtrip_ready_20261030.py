"""Publish one concrete operator action after all model/source checks pass."""
import datetime as dt,json
from pathlib import Path
import numpy as np
from yaw_roundtrip_plan_20261030 import ROOT,OUT,REQUEST,PROOF,LOADED,HAND_PATH,YawRoundtripPlan,sha
from yaw_roundtrip_gate_20261030 import validate_review,run_screen
from observed_book_model_policy import healthy_off
from hand_motor_mapping import SIM_TO_MOTOR_IDS
def main():
    p=YawRoundtripPlan(REQUEST);proof=validate_review(p,PROOF)
    current=ROOT/'fast_off_readonly_20261030_010207_418003/result.json';r=json.loads(current.read_bytes())
    assert r['status']=='off_snapshots_saved' and len(r['snapshots'])==3 and r['maximum_drift_ticks']<=2 and r['servo_writes']==r['torque_changes']==0
    for s in r['snapshots']:healthy_off({k:np.asarray(v) for k,v in s.items()})
    difference=np.asarray(r['snapshots'][-1]['position'])-p.source_position()
    assert max(abs(difference))<=4,'New current EMPTY prefix review required before publishing'
    live=OUT/'latest_off_request.json';cached=OUT/'latest_off_review.json'
    live.write_text(json.dumps(dict(plan_version=p.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=r['snapshots'][-1],
        source=current.relative_to(ROOT).as_posix(),source_sha256=sha(current),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    run_screen(live,cached);validate_review(YawRoundtripPlan(live),cached)
    protected={
        'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
        'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
        'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
        'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d',
        'horizontal_quarter_motion_runner_20261027.py':'5c8487f134264f5ef4eb9f6bdf6a465e5fb6c69945c4a91407e1b6d75cc760cb',
        'screen_material_checked_reverse_cube_20261029.py':'79adde54e2d3a8e45bbd895b0f50b5c297536c0556ff508e39b54f567e3a3d66'}
    for name,digest in protected.items():assert sha(ROOT/name)==digest,('Protected file changed',name)
    api=Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py')
    assert sha(api)=='008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'
    models=proof['composed_cube_models'];now=dt.datetime.now().astimezone().isoformat();command='leap_yaw_roundtrip_trial.cmd'
    report=dict(status='supervised_continuous_forward_inverse_trial_ready',timestamp=now,command=command,
        proof_sha256=sha(PROOF),candidate_sha256=sha(LOADED),hand_path_sha256=sha(HAND_PATH),cube_cases=27,composed_finite_hand_cases=9,
        current_prefix_static_cases=201,current_prefix_dynamic_cases=9,software_admission_tests_passed=16,default_no_devices_checked=True,
        current_readonly=current.relative_to(ROOT).as_posix(),current_readonly_sha256=sha(current),current_off_difference_ticks=difference.tolist(),current_pose_cache_checked=True,
        all16_OFF_error0_watchdog0_verified=True,protected_sources_unchanged=True,assistant_motor_or_setting_writes=0,
        loaded_goals=2627,whole_goals=len(p.plan(p.source_position())),nominal_motion_s=proof['nominal_motion_seconds'],unchanged_motion_limit_s=300,
        new_joint_goal_vectors=0,physical_forward_and_reverse_segments_were_separately_executed=True,entire_composed_cycle_not_yet_physically_tested=True,
        historical_forward_placed_to_held_deg=88.71186086891521,historical_forward_grasp_to_held_deg=70.9070281935347,
        historical_forward_angle_includes_grasp=True,inverse_retraces_only638_yaw_goals=True,inverse90_ready=False,
        modeled_forward_nominal_deg=models['nominal_forward_deg'],modeled_inverse_nominal_deg=models['nominal_inverse_deg'],modeled_inverse_range_deg=models['inverse_range_deg'],
        metric_robot_registration_verified=False,latest_physical_run='supported_inverse_trial_20261029_231345_341471',latest_physical_inverse_estimate_deg=-32.376021450567855,
        rejected_extension_candidates_not_run=True,rejected_unseated_models_preserved=True,
        guide=(OUT/'operator_guide.html').relative_to(ROOT).as_posix(),placement='GREEN up; ORANGE toward wrist ID41',
        ready_for_supervised_operator_run=True,operator_action_required=True,
        operator_steps=['FIRST terminal SPACE with EMPTY hand and cube outside camera.',
            'At PLACE set GREEN up / ORANGE wristward naturally supported; remove hands; SECOND SPACE.',
            'Forward and inverse continue without a new placement. Keep hands clear between legs.',
            'After automatic OPEN and REMOVE, remove Cube; THIRD terminal SPACE ends OFF/restoration.'],
        physical_review_required='Measure forward placed-to-held, forward grasp-to-held, inverse first-held-to-second-held, and final OPEN separately. The638-goal inverse is NOT automatically a90-degree inverse.')
    (OUT/'handoff.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    archive=OUT/'previous_status_pointers';archive.mkdir(exist_ok=True)
    for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        path=ROOT/name;old=json.loads(path.read_bytes());saved=archive/path.name
        if not saved.exists():saved.write_bytes(path.read_bytes())
        old.update(status=report['status'],current_status=report['status'],timestamp=now,next_user_command=command,next_command=command,next_user_command_ready=True,
            next_user_action_required=True,next_user_action='Run continuous forward/inverse observation: EMPTY first SPACE; PLACE GREEN up/ORANGE wristward and second SPACE; keep Cube in place between legs; automatic OPEN then REMOVE and third SPACE/OFF.',
            next_result_prefix='yaw_roundtrip_trial_',ready_for_user_run=True,ready_for_supervised_operator_run=True,ready_scope='CONTINUOUS_TWO_LEG_OBSERVATION_NOT_INVERSE90_CERTIFICATE',
            automatic_motion_started=False,assistant_motor_or_setting_writes=0,current_force_and_health_criteria_unchanged=True,current_ma=450,sample_period_s=.075,goal_step_ticks=2,
            active_motion_limit_s=300,nominal_motion_seconds=report['nominal_motion_s'],trajectory=LOADED.relative_to(ROOT).as_posix(),
            cube_orientation_at_place=report['placement'],placement_guide=report['guide'],proof=PROOF.relative_to(ROOT).as_posix(),proof_sha256=sha(PROOF),
            latest_readonly=report['current_readonly'],source_off_readonly=json.loads(REQUEST.read_bytes())['source'],latest_pose_difference_from_review_ticks=difference.tolist(),
            source_off_drift_ticks=0,motors_off_verified=True,latest_start_proof=cached.relative_to(ROOT).as_posix(),
            latest_camera_inventory='contact_view_readonly_20261030_001756_861621/review.json',expected_camera_present=True,
            model_inverse=models,initial_cube_pose_not_registered=True,new_pose_geometry_cases=0,new_whole_hand_dynamics_cases=9,current_empty_recovery_static_cases=201,
            current_empty_recovery_dynamic_cases=9,software_checks_passed=16,old_programs_and_poster_unchanged=True,original_cube_source_and_guard_files_verified=True,no_live_SAC=True,
            inverse_cube_trial_ready=False,new_supported_inverse_cube_trial_not_yet_run=False,inverse_cube_physical_trial_not_yet_run=False,
            roundtrip_trial_ready=True,roundtrip_physical_trial_not_yet_run=True,physical_roundtrip_verified=False,inverse_90_ready=False,all_directions_90_verified=False,
            roundtrip_handoff=(OUT/'handoff.json').relative_to(ROOT).as_posix(),current_motor_connection='All16 OFF/error0/watchdog0; three identical fresh reads and <=4tick cached prefix verified',
            inverse_research_progress='Fresh contact-extension hypotheses were not dynamically effective. Next separates placement effects by continuous forward/inverse; historical89 includes18deg grasp and71deg yaw.')
        path.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    marker='<!-- continuous_yaw_ready_20261030 -->\n'
    paragraph=('2026-10-30 다음 실물 작업: `leap_yaw_roundtrip_trial.cmd` 연속 정회전/역회전 관찰 시험을 준비했다. '
        '현재 모터16개 OFF와 오류0, 3회 같은 관절값, 최신 복귀 캐시를 읽기만 확인했다. 현재 복귀 기하201/유한9, 전체 빈손 유한9, 큐브 포함1ms 유한27조건과 소프트웨어 검증16개를 통과했다. '
        '새 관절 목표 좌표는 없으며, 이전에 실제 실행한 두 경로를 연결했다. 기존450mA/PID/75ms와 모든 중단 한도는 유지한다. '
        '첫 SPACE는 빈손, PLACE에서 초록 위/주황 손목 ID41쪽으로 한 번 놓고 손을 빼고 두 번째 SPACE, 정→역 사이에는 큐브를 그대로 두며, 자동 OPEN 뒤 REMOVE에서 큐브를 빼고 세 번째 SPACE로 OFF한다. 동작 약4분이다.\n\n'
        '각도 기준을 구분했다. 이전 약88.71도는 잡는 동작 약17.80도와 회전638지령 약70.91도를 합친 값이다. 이번에는 그638지령만 거꾸로 가므로 역회전90도 성공으로 간주하면 안 된다. '
        '새 모델의 명목 정회전은61.76도, 역회전은-30.41도이며 역회전 모델 범위는-62.59~-26.12도다. 실제 새 왕복은 아직 실행하지 않았다. '
        '마지막 실제 큐브 시험은10/29 23:13의 약32.38도이며, 접촉 보완 IK 후보는 물리 계산에서 추가 회전이 나오지 않아 실행하지 않았다. '
        '초기 겹침으로 거부한27조건의 원본도 보존했다. 명령과 근거: right_hand_rotation_20261030/yaw_roundtrip/handoff.json\n\n')
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig')
        if not old.startswith(marker):path.write_text(marker+paragraph+old,encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('operator_steps','physical_review_required')},ensure_ascii=False))
if __name__=='__main__':main()
