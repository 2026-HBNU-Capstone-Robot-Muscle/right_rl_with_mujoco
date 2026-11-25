"""Publish the concrete nominal supervised trial without claiming robustness."""
import datetime as dt,hashlib,json
from pitch_slow_yaw_plan_20261029 import ROOT,OUT,REQUEST,PROOF,PitchSlowYawPlan
from pitch_slow_yaw_gate_20261029 import validate_review
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    p=PitchSlowYawPlan(REQUEST);review=validate_review(p,PROOF)
    protected={
        'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
        'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
        'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
        'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d'}
    for name,value in protected.items():assert sha(ROOT/name)==value,name
    cube=json.loads((OUT/'loaded_cube_evidence.json').read_bytes())
    inventory=json.loads((OUT/'camera_inventory.json').read_bytes())
    command=ROOT/'leap_pitch_slow_yaw.cmd'
    model=ROOT/'right_hand_rotation_20261028/two_axis_connection/actual_handoff_yaw_diagnostic/whole_pitch_handoff_yaw_hypothesis/face_states.json'
    state=dict(status='ready_for_user_started_nominal_supervised_pitch_then_slower_yaw_trial',
        timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat(),next_user_command=str(command),
        next_user_command_ready=True,next_user_action_required=True,
        next_user_action=('Run the new combined command once, starting with an empty hand.' if inventory['expected_camera_present'] else
                          'Reconnect the Streaming Webcam USB for the next distinct physical trial.'),
        expected_camera_present=inventory['expected_camera_present'],latest_camera_inventory=inventory,
        source_off_readonly='prefix638_off_readonly_20261029_001123_114252',source_off_drift_ticks=1,
        motors_off_verified=True,assistant_motor_or_setting_writes=0,
        trajectory='EMPTY preparation -> PLACE once -> demonstrated corrected pitch -> retained transfer -> slower yaw -> OPEN -> REMOVE -> OFF',
        cube_orientation_at_place='GREEN up; ORANGE toward WRIST/ID41',cube_reposition_between_rotations_required=False,
        nominal_motion_seconds=review['nominal_motion_seconds'],active_motion_limit_s=480,old_program_active_limits_still300=True,
        native_ranges_current_PID_voltage_tracking_temperature_criteria_unchanged=True,angular_reference_targets_unchanged=True,
        yaw_distinct_goal_period_s=.15,ordinary_cycle_s=.075,goal_step_ticks=2,current_ma=450,
        maximum_aggregate_thumb_displacement_ticks=1568,
        nominal_full_trajectory_cases_passed=3,broad_parameter_cases_passed=cube['broad_parameter_cases_passed'],broad_parameter_cases=27,
        wide_parameter_robustness_verified=False,supervised_nominal_experiment_only=True,physical_combined_rotation_verified=False,
        nominal_retained_pose_neighborhood_cases=9,full_loaded_hand_dynamics_cases=9,empty_recovery_cases=210,software_checks_passed=13,
        nominal_model_pitch_red_up_error_deg=.1303642669841653,
        nominal_model_yaw_from_pitch_hold_deg=95.83936160128299,nominal_model_yaw_after_open_deg=85.05313330316555,
        real_cube_pose_registration_verified=False,
        proof=PROOF.relative_to(ROOT).as_posix(),proof_sha256=sha(PROOF),model_face_states=model.relative_to(ROOT).as_posix(),
        latest_actual_pitch_handoff_run='pitch_handoff_observe_20261028_221727_181336',
        latest_actual_pitch_handoff_late_placement_recorded=True,
        old_programs_and_poster_unchanged=True)
    write(OUT/'readiness.json',state)
    comparison=OUT/'latest_readonly_comparison.json'
    if comparison.exists():
        latest=json.loads(comparison.read_bytes())
        state.update(latest_readonly=latest['source'],latest_pose_difference_from_review_ticks=latest['maximum_difference_from_review_ticks'],latest_pose_cache_verified=latest['cached_review_verified'])
        write(OUT/'readiness.json',state)
    for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
                 'right_hand_rotation_20261026/horizontal_next_step_20261026.json',
                 'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        path=ROOT/name
        old=OUT/(path.stem+'_before_ready.json')
        if not old.exists() and path.exists():old.write_bytes(path.read_bytes())
        write(path,state)
    guide='''이번 파일은 큐브를 한 번 놓고 뒤집기와 느린 수평 회전을 연결하는 감독 실험용입니다.

기본 모델에서는 끝까지 동작했고, 수평 회전은 고정 중 약96도와 손을 편 뒤 약85도로 나왔습니다. 넓힌 구동 세기와 마찰 조건에서는27개 중3개만 완주했습니다. 조건이 달라도 안정적으로 성공하는 완성본이나 실물 연결 성공으로 인증하지 않습니다. 실패 기록도 그대로 보존했습니다.

웹캠 Streaming Webcam을 다시 연결하고 아래 명령을 한 번 실행합니다. 다른 모터 프로그램은 닫습니다. 손가락과 받침을 손으로 다시 맞추지 않습니다.

```powershell
& "'''+str(command)+'''"
```

1. 큐브를 로봇 손과 카메라 밖에 둔 빈손 상태에서 터미널 첫 SPACE를 누릅니다. 먼저 준비 자세를 잡습니다.
2. `[큐브 놓기]`가 표시되어 손이 정지해 기다릴 때 큐브를 놓습니다. 초록이 위, 주황이 손목 ID41 쪽입니다. 아래 이전 성공 시작 사진을 참고합니다. 큐브가 스스로 놓인 것을 확인하고 사람 손을 완전히 뺀 뒤 터미널에서 두 번째 SPACE를 누릅니다.
3. 약7~8분 동안 뒤집기, 다음 자세 이동, 느린 수평 회전과 자동 손 펴기가 이어집니다. 중간에 큐브를 다시 놓거나 잡지 않습니다.
4. `[REMOVE CUBE]`가 표시되면 큐브를 빼고, 뺀 뒤에만 세 번째 SPACE를 눌러 종료합니다. Q/Esc/Ctrl+C는 중단입니다.

이전 성공 배치 사진: [초록 위 / 주황 손목 쪽]('''+(ROOT/'loaded_thumb_rotation_20261026_135418_442474/cube_before_turn.jpg').as_posix()+''')

각도 목표는 이전 파일과 같습니다. 수평 회전 목표를75ms 한 번 더 유지해 서로 다른 수평 목표 사이를150ms로 늘렸습니다.450mA와 기존PID, 관절 범위, 전압/온도/추종 정지 조건은 유지합니다. 두 회전을 연결하는 이 파일의 총 구동 시간 제한만480초로 따로 검토했고, 이전 단일 동작 파일의300초 제한은 유지합니다. 온라인SAC 동작은 아닙니다.

처음과 각 고정 구간, 끝 사진과 관절 기록은 자동 저장됩니다. 완료 후 ‘했어’만 알려주면 됩니다.
'''
    (OUT/'실행안내.md').write_text(guide,encoding='utf-8')
    note='''2026-10-29 다음 사용자 작업: Streaming Webcam USB를 다시 연결해 새 통합 실물 실험을 촬영할 준비를 한다. 카메라 목록에는 내장USB2.0과OBS만 있어 별도 웹캠은 현재 없다. 모터는00:11에1Mbps에서16개를 읽기만 해 모두OFF/오류0으로 확인했고,3회 각도 차이는 최대1틱이었다. 모터/토크/설정 쓰기는0회다.

`leap_pitch_slow_yaw.cmd`는 빈손 준비→PLACE 한 번→보완 뒤집기→자세 이동→느린 수평 회전→자동OPEN→REMOVE/OFF를 연결하는 감독 실험용이다. 원래의 각도 목표는 바꾸지 않고 수평 구간만 같은 목표를 한 주기 더 유지한다. 총5790목표, 명목434.25초다. 이 파일의 구동 시간 제한은480초로 별도 검토했고 기존 단일 파일의300초와450mA/PID/관절/전압/온도/추종 기준은 유지했다.

중간에 큐브 위치를 강제로 다시 지정하지 않은 전체 물리 모델의 기본 조건3개는 끝까지 통과했다. 모델에서 빨강 윗면이 된 뒤 수평 회전은 고정 약95.84도와OPEN 후85.05도다. 다만 구동 세기와 마찰을 넓힌27조건 중3개만 통과했으므로 안정적 제어 완성이나 넓은 조건 검증으로 표시하지 않는다. 실패24조건도 보존했다. 별도 기본 조건의 큐브 방향±6도/앞뒤±3mm 아홉 조건은 느린 수평 동작을 통과했다. 전체 손 동역학9조건 및 현재 빈손 복귀210조건, 제어/기본실행 검사13개도 통과했다. 실제로 두 회전을 연달아 성공한 것은 아직 확인하지 않았다.

자료는 `right_hand_rotation_20261029/pitch_then_slow_yaw_v1/readiness.json`과`review.json`, `loaded_cube_evidence.json`에 있다. 코드 검토는 기본 조건의 감독 실험 범위를 명시적으로 사용하며 넓은 조건이 모두 통과했다는 플래그를 허용하지 않는다. 새 큐브 위치는 계측 인증하지 않았다. 이전 실물 수직 성공2회와 보완 뒤집기/전달 관찰1회, 수평 회전 기록을 보존했다. 전시 문구와 원본 경로/공통 런타임은 바꾸지 않았다.

<!-- END NOMINAL CONTINUOUS TRIAL READY 20261029 -->

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig')
        if '<!-- END NOMINAL CONTINUOUS TRIAL READY 20261029 -->' not in old:path.write_text(note+old,encoding='utf-8')
    if inventory['expected_camera_present']:
        connected='''2026-10-29 00:42 사용자 웹캠 연결 확인 완료. Streaming Webcam이 다시 목록에 있고, 모터는3회 읽기 모두OFF/오류0 및 최대1틱 변화였다. 준비 경로의 검토 중심과도 최대1틱 차이여서 기존4틱 범위의 검토를 그대로 검증해 재사용했다. 모터나 카메라 스트림을 구동하지 않았다. 현재 다음 사용자 작업은 새 `leap_pitch_slow_yaw.cmd` 한 번이다. 빈손 첫SPACE, PLACE에서 초록 위/주황 손목 ID41 쪽 배치와 손 제거 후 두 번째SPACE, 약7~8분 뒤 REMOVE에서 큐브 제거 후 세 번째SPACE다.

이것은 기본 조건의 감독 실험용 연결 파일이며 넓은 구동/마찰 조건 검증이나 실물 연속 성공을 완료로 보고하지 않는다. 자세한 범위와 원본 실패 기록은 바로 아래 기록 및 `right_hand_rotation_20261029/pitch_then_slow_yaw_v1`에 있다.

<!-- END CONNECTED CONTINUOUS TRIAL READY 20261029 -->

'''
        for name in ('professor_progress_20261015.md','sac_current_status.md'):
            path=ROOT/name;old=path.read_text(encoding='utf-8-sig')
            if '<!-- END CONNECTED CONTINUOUS TRIAL READY 20261029 -->' not in old:path.write_text(connected+old,encoding='utf-8')
    # Correct the label of the earlier post-integration diagnostic, preserving
    # its values and distinguishing it from the later precise contact snapshots.
    old_render=ROOT/'right_hand_rotation_20261028/two_axis_connection/actual_handoff_yaw_diagnostic/visual_review/render_review.json'
    if old_render.exists():
        r=json.loads(old_render.read_bytes())
        if 'exact_failed_state_contacts' in r:
            r['post_integration_saved_state_contacts']=r.pop('exact_failed_state_contacts')
            r['these_snapshot_contacts_are_not_the_maximum_failure_contact_state']=True;write(old_render,r)
    print(json.dumps(dict(status=state['status'],camera_connected=state['expected_camera_present'],
        command=str(command),tests=13,supervised_nominal_only=True,broad_parameter_passes=3,broad_cases=27,devices_opened=False),ensure_ascii=False))
if __name__=='__main__':main()
