"""Publish a reviewable operator handoff after proof, read-only and integrity checks."""
import datetime as dt,json
from pathlib import Path
import numpy as np
from gapped_inverse_plan_20261029 import GappedInversePlan,ROOT,OUT,REQUEST,PROOF,sha
from gapped_inverse_gate_20261029 import validate_review,run_screen
from hand_motor_mapping import SIM_TO_MOTOR_IDS

PROTECTED={
    ROOT/'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
    ROOT/'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
    ROOT/'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
    ROOT/'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d',
    ROOT/'horizontal_quarter_motion_runner_20261027.py':'5c8487f134264f5ef4eb9f6bdf6a465e5fb6c69945c4a91407e1b6d75cc760cb',
    Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'):'008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656',
}

def main():
    for path,expected in PROTECTED.items():
        if sha(path)!=expected:raise RuntimeError('Protected source differs: '+str(path))
    plan=GappedInversePlan(REQUEST);proof=validate_review(plan,PROOF)
    readonly=ROOT/'fast_off_readonly_20261029_183632_339839/result.json';read=json.loads(readonly.read_bytes())
    assert read['status']=='off_snapshots_saved' and read['maximum_drift_ticks']<=2 and read['servo_writes']==0 and read['torque_changes']==0
    for state in read['snapshots']:plan.validate_off_state(state)
    raw=np.asarray(read['snapshots'][-1]['position'],int)
    maximum_difference=int(max(abs(raw-plan.source_position())))
    if maximum_difference>4:raise RuntimeError('Latest pose needs a new current-prefix model before publication')
    live=OUT/'live_start_183632';live.mkdir(exist_ok=True)
    request=live/'request.json';review=live/'review.json'
    request.write_text(json.dumps(dict(plan_version=plan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
        off_state=read['snapshots'][-1],source=readonly.relative_to(ROOT).as_posix(),
        source_sha256=sha(readonly),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    current=GappedInversePlan(request)
    assert current.review_center_reused
    run_screen(request,review)
    camera=json.loads((OUT/'current_camera_inventory.json').read_bytes())
    assert camera['expected_camera_present'] and not camera['camera_stream_opened'] and not camera['motor_api_used']
    command='& "'+str(ROOT/'leap_gapped_inverse_trial.cmd')+'"'
    stamp=dt.datetime.now().astimezone().isoformat()
    status=dict(status='gapped_inverse_operator_observation_ready',current_status='gapped_inverse_operator_observation_ready',
        timestamp=stamp,goal='오른손으로 원하는 방향 약90도 회전; 전체 목표는 진행 중',
        next_user_command=command,next_command=command,next_user_command_ready=True,
        next_user_action_required=True,next_user_action='새 명령을 한 번 실행. 첫 SPACE는 빈손, PLACE에서 초록 위와 흰 면 손목 ID41 쪽, 손을 뺀 뒤 두 번째 SPACE. 자동 OPEN 후 REMOVE에서 큐브를 빼고 세 번째 SPACE.',
        next_result_prefix='gapped_inverse_trial_',ready_for_user_run=True,
        ready_for_supervised_operator_run=True,automatic_motion_started=False,assistant_motor_or_setting_writes=0,
        current_force_and_health_criteria_unchanged=True,current_ma=450,sample_period_s=.075,
        goal_step_ticks=2,active_motion_limit_s=300,nominal_motion_seconds=len(plan.plan(plan.source_position()))*.075,
        trajectory='EMPTY recovery -> EMPTY reverse preposition -> small placement gap -> PLACE -> slow close -> reverse638 -> OPEN -> REMOVE/OFF',
        cube_orientation_at_place='GREEN up; WHITE toward WRIST/ID41',
        placement_guide='right_hand_rotation_20261029/gapped_inverse_cube/placement_views.png',
        latest_actual_run='reverse_yaw_empty_20261029_164720_101745',
        latest_actual_review='reverse_yaw_empty_20261029_164720_101745/offline_review/review.json',
        last_actual_cube_rotation_review='adaptive_yaw_loaded_20261029_160451_416563/offline_review/physical_review.json',
        last_actual_cube_video='adaptive_yaw_loaded_20261029_160451_416563/offline_review/full_run_realtime.mp4',
        inverse_cube_physical_trial_not_yet_run=True,physical_inverse_cube_rotation_verified=False,
        inverse_90_ready=False,all_directions_90_verified=False,physical_combined_rotation_verified=False,
        proof=PROOF.relative_to(ROOT).as_posix(),proof_sha256=sha(PROOF),
        latest_readonly=readonly.relative_to(ROOT).as_posix(),source_off_readonly=readonly.relative_to(ROOT).as_posix(),
        latest_pose_difference_from_review_ticks=maximum_difference,source_off_drift_ticks=read['maximum_drift_ticks'],
        motors_off_verified=True,latest_start_proof=review.relative_to(ROOT).as_posix(),
        latest_camera_inventory=camera,expected_camera_present=True,
        model_inverse=proof['inverse_model'],initial_cube_pose_not_registered=True,
        new_pose_geometry_cases=201,new_whole_hand_dynamics_cases=9,
        current_empty_recovery_static_cases=201,current_empty_recovery_dynamic_cases=9,
        software_checks_passed=7,old_programs_and_poster_unchanged=True,
        original_cube_source_and_guard_files_verified=True,no_live_SAC=True)
    pointers=[ROOT/'right_hand_rotation_20261028/two_axis_connection/current_progress.json',
        ROOT/'right_hand_rotation_20261026/horizontal_next_step_20261026.json',
        ROOT/'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json']
    archive=OUT/'previous_status_pointers';archive.mkdir(exist_ok=True)
    for path in pointers:
        target=archive/(path.stem+'.json')
        if not target.exists():target.write_bytes(path.read_bytes())
        payload=dict(status,previous_status_archive=target.relative_to(ROOT).as_posix())
        path.write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (OUT/'handoff.json').write_text(json.dumps(status,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block=('2026-10-29 역회전 관찰 준비 완료. 16:47 빈손 역방향 실행은 2601개 지령 완료, 최저4.8V, 최대96mA, 최대추종0.0813rad, OFF와설정복구를 확인했다. '
        '2회 읽기 오류는 기존200ms 안에서 정상 전체상태2회로 복구했으며 목표 재전송은 없었다. '
        '큐브가 있는 역순 경로는 처음 쥐는 구간이 회전을 상쇄해 제외했다. 그 구간을 빈손으로 먼저 만들고, 엄지1번을5.625도,2번을2.8125도,중지11번을1.40625도 열어 큐브를 놓을 공간을 만들었다. '
        '놓은 뒤32개 목표로 천천히 닫고 기존 빈손 완주와 같은638개 역방향 회전 목표를 실행한다. '
        '실제 양쪽 접촉 마찰0.2/0.3/0.5, 구동강도0.5/1/1.5, 받침3조건의 총27개1ms 유한강성 모델에서 놓침과 기존 한계 초과 없이 약26.65~39.01도 역회전했으며 명목31.23도다. '
        '이 값은 실물 확인 전이고 역방향90도 준비 완료를 뜻하지 않는다. 과거 모델의 손끝 마찰0.7이 큐브 마찰보다 크면 손끝 접촉에0.7이 적용되므로, 이번 비교는 손과큐브 양쪽에 같은 계수를 적용하고 실제 접촉값을 확인했다. 물리 재료를 측정한 값은 아니다. '
        '새32개 자세201조건, 전체 빈손 경로9조건, 현재 복귀201정적+9동적, 소프트웨어7검사를 통과했다. 기존 수동OFF 엄지ID4 복귀 예외를 유지했으며 새로운 하한이나 건강 기준을 추가로 완화하지 않았다. '
        '18:36 읽기 전용3회 검사에서 모두OFF,드리프트1틱,검토 출발점차이'+str(maximum_difference)+'틱을 확인했고 Streaming Webcam도 열거됐다. '
        '다음 leap_gapped_inverse_trial.cmd: 첫SPACE빈손,PLACE뒤초록위/흰면손목ID41,손을뺀뒤두번째SPACE,자동OPEN/REMOVE뒤큐브빼고세번째SPACE. 큐브가 자연스럽게 놓이지 않으면Q. '
        '배치 그림 right_hand_rotation_20261029/gapped_inverse_cube/placement_views.png, 검토 operator_review.json, 새 결과폴더 gapped_inverse_trial_*. '
        '원본시퀀스/네이티브설정/모션러너/족자는해시보존. 도구로 모터 쓰기는 하지 않았다. 이전 상태기록은 아래에 보존한다.\n\n')
    marker='<!-- gapped_inverse_handoff_20261029 -->\n'
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8-sig')
        if not text.startswith(marker):path.write_text(marker+block+text,encoding='utf-8')
    print(json.dumps(dict(status=status['status'],next_user_command=command,
        latest_start_difference_ticks=maximum_difference,software_checks_passed=7,
        material_cases=27,physical_inverse_verified=False,assistant_motor_writes=0),ensure_ascii=False))

if __name__=='__main__':main()
