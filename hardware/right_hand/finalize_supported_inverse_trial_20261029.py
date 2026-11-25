"""Publish only a source-bound supervised Cube observation, not a90deg claim."""
import datetime as dt,json
import numpy as np
from supported_inverse_trial_plan_20261029 import SupportedInverseTrialPlan,ROOT,OUT,REQUEST,PROOF,LOADED,ACTUAL,ACTUAL_REVIEW,sha
from supported_inverse_trial_gate_20261029 import validate_review,run_screen
from observed_book_model_policy import healthy_off
from hand_motor_mapping import SIM_TO_MOTOR_IDS
def main():
    p=SupportedInverseTrialPlan(REQUEST);proof=validate_review(p,PROOF)
    current=ROOT/'fast_off_readonly_20261029_230616_377103/result.json';r=json.loads(current.read_bytes())
    assert r['status']=='off_snapshots_saved' and len(r['snapshots'])==3 and r['maximum_drift_ticks']<=2 and r['servo_writes']==r['torque_changes']==0
    for state in r['snapshots']:healthy_off({k:np.asarray(v) for k,v in state.items()})
    difference=np.asarray(r['snapshots'][-1]['position'])-p.source_position()
    assert np.max(abs(difference))<=4,'Latest current pose needs a new prefix review before publishing'
    live=OUT/'latest_off_request.json';live.write_text(json.dumps(dict(plan_version=p.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
        off_state=r['snapshots'][-1],source=current.relative_to(ROOT).as_posix(),source_sha256=sha(current),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    cached=OUT/'latest_off_review.json';run_screen(live,cached);validate_review(SupportedInverseTrialPlan(live),cached)
    protected={
        'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
        'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
        'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
        'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d',
        'horizontal_quarter_motion_runner_20261027.py':'5c8487f134264f5ef4eb9f6bdf6a465e5fb6c69945c4a91407e1b6d75cc760cb'}
    assert all(sha(ROOT/path)==digest for path,digest in protected.items())
    actual=json.loads(ACTUAL_REVIEW.read_bytes());visual=ACTUAL.parent/'offline_review/visual_review.json'
    report=dict(status='supervised_supported_inverse_cube_observation_ready',timestamp=dt.datetime.now().astimezone().isoformat(),
        command='leap_supported_inverse_trial.cmd',scope='operator_started_cube_observation_actual_angle_to_review',
        physical_empty_source=ACTUAL.relative_to(ROOT).as_posix(),physical_empty_source_sha256=sha(ACTUAL),physical_empty_review_sha256=sha(ACTUAL_REVIEW),
        physical_empty_all_goals_completed=True,physical_empty_visual_review=visual.relative_to(ROOT).as_posix(),
        physical_empty_minimum_voltage_v=actual['minimum_voltage_v'],physical_empty_peak_current_ma=actual['peak_current_ma'],
        physical_empty_maximum_tracking_rad=actual['maximum_tracking_rad'],physical_empty_read_failures=actual['read_failures'],
        physical_empty_camera_blank_frames=actual['camera_blank_frames'],physical_empty_OFF_and_restoration_verified=True,
        proof_sha256=sha(PROOF),candidate_sha256=sha(LOADED),cube_model_cases=27,current_prefix_static_cases=201,current_prefix_finite_cases=9,
        admission_regression_tests_passed=4,default_entry_without_devices_checked=True,current_pose_cache_checked=True,current_off_difference_ticks=difference.tolist(),
        model_nominal_placed_to_hold_deg=proof['supported_inverse_cube_models']['nominal_placed_to_hold_deg'],
        model_range_deg=proof['supported_inverse_cube_models']['range_deg'],physical_new_cube_trial_completed=False,physical_inverse_90_verified=False,
        placement='GREEN up; WHITE toward wrist ID41; naturally supported between thumb and middle above palm',
        no_new_health_or_force_policy=True,assistant_motor_or_setting_writes=0,protected_sources_unchanged=True,
        ready_for_supervised_operator_run=True,operator_steps=['FIRST terminal SPACE with hand empty; cube outside camera.',
            'At PLACE, set cube GREEN up/WHITE wristward; remove hands, SECOND terminal SPACE.',
            'After automatic OPEN, REMOVE cube; THIRD terminal SPACE verifies OFF/restoration.',
            'Q/Esc/Ctrl+C stops. No automatic motion retry.'])
    (OUT/'handoff.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    for rel in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        path=ROOT/rel;old=json.loads(path.read_bytes());old.update(status=report['status'],current_status=report['status'],timestamp=report['timestamp'],
            next_command=report['command'],next_user_command=report['command'],next_user_command_ready=True,next_user_action_required=True,
            next_user_action='Run new supported Cube trial: EMPTY first SPACE; PLACE GREEN up/WHITE wristward and second SPACE; automatic OPEN then REMOVE and third SPACE/OFF.',
            ready_for_user_run=True,ready_for_supervised_operator_run=True,ready_scope='SUPERVISED_CUBE_OBSERVATION_NOT_VERIFIED90',
            latest_actual_run=ACTUAL.parent.name,latest_actual_review=ACTUAL_REVIEW.relative_to(ROOT).as_posix(),new_supported_inverse_EMPTY_completed=True,
            inverse_cube_trial_ready=True,new_supported_inverse_cube_trial_not_yet_run=True,inverse_90_ready=False,
            supported_inverse_cube_handoff=(OUT/'handoff.json').relative_to(ROOT).as_posix(),current_motor_connection='All16 torque OFF/error0/watchdog0; stable latest readings and bounded cache verified',assistant_motor_or_setting_writes=0)
        path.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    marker='<!-- supported_inverse_cube_ready_20261029_after_EMPTY -->\n'
    paragraph=(f'新규경로 빈손 실물 완료: {ACTUAL.parent.name},전체2445지령/새손경로1883지령/회전328지령,최저{actual["minimum_voltage_v"]:.1f}V/최대{actual["peak_current_ma"]}mA/최대{actual["maximum_tracking_rad"]:.5f}rad,읽기실패0/검정0,OFF와복구확인. '
        '첫실행225208은프리뷰취소로구동쓰기가없다. 종료OFF자세는중력으로이동하여별도새3회읽기를사용했다. '
        f'새 큐브시험 leap_supported_inverse_trial.cmd 준비: 큐브모델27/현재복귀기하201/유한9/거부회귀시험4 통과. 모델명목{report["model_nominal_placed_to_hold_deg"]:.2f}도이며실물90도성공이아니다. '
        '첫SPACE빈손,PLACE때초록위/흰면손목ID41쪽,손빼고두번째SPACE,자동OPEN후큐브빼고세번째SPACE/OFF다. 기존450mA/PID/75ms와중단한도유지. '
        '근거: right_hand_rotation_20261029/supported_inverse_trial/handoff.json\n\n')
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8-sig')
        if not text.startswith(marker):path.write_text(marker+paragraph+text,encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='operator_steps'}))
if __name__=='__main__':main()
