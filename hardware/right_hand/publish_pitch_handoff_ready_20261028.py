"""Publish one concrete user action after evidence and operator checks pass."""
import datetime as dt,hashlib,json
from pitch_handoff_plan_20261028 import ROOT,OUT,REQUEST,PROOF,PitchHandoffPlan
from pitch_handoff_gate_20261028 import validate_review

def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    proof=validate_review(PitchHandoffPlan(REQUEST),PROOF)
    current=json.loads((OUT/'last_readonly_pose_comparison.json').read_bytes())
    assert current['review_center_reused'] is True
    protected={
        'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
        'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
        'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
        'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d'}
    for name,expected in protected.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected,name
    command=ROOT/'leap_pitch_handoff_observe.cmd'
    state=dict(status='ready_for_one_user_started_pitch_then_transfer_observation',
        timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat(),
        next_user_command=str(command),next_user_command_ready=True,
        user_start_requires_empty_hand=True,cube_orientation='GREEN up / ORANGE toward wrist ID41',
        next_loaded_action='Recorded pitch with IF6-8/RF14+8 clearance, then600 slow transfer goals. No subsequent yaw.',
        loaded_pitch_correction_degrees=[-.703125,.703125],motor_ids=[6,14],other14_pitch_joint_targets_unchanged=True,
        nominal_active_motion_seconds=233.925,nominal_empty_preparation_seconds=90.825,
        model_cases=dict(empty_static=201,empty_dynamic=9,loaded_static=201,loaded_hand_dynamic=9,conditional_transfer=27),
        software_checks_passed=13,actual_hardware_read_confirmed=True,
        latest_readonly=current['source'],latest_off_position_drift_from_review_max_ticks=1,
        motors_off_verified=True,measured_off_voltage_v=[5.0,5.1],measured_temperature_c=20,
        old_yaw_experiment_unchanged=True,original_trajectory_and_poster_unchanged=True,
        review=PROOF.relative_to(ROOT).as_posix(),review_sha256=hashlib.sha256(PROOF.read_bytes()).hexdigest(),
        reviewed_center_noise_radius_ticks=4,actual_enable_radius_still4=True,
        aggregate_thumb_displacement_guard_for_this_two_setup_macro_ticks=1568,
        old_yaw_aggregate_guard_ticks=1300,native_joint_ranges_not_extended=True,
        current_gain_voltage_tracking_temperature_stops_unchanged=True,
        existing_palm7_model_seam_policy_reused_not_widened=True,
        physical_corrected_pitch_unverified=True,physical_transfer_unverified=True,
        next_yaw_forbidden=True,hardware_execution_performed_by_assistant=False,
        robot_usb_adapter_currently_missing=False)
    write(OUT/'readiness.json',state)
    parent=OUT.parent;write(parent/'current_progress.json',state)
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
                 'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        write(ROOT/name,state)
    guide='''로봇 USB와5V 전원, 웹캠을 연결한 상태에서 아래 명령을 한 번 실행한다. 손가락과 받침은 그대로 두고 시작 시 큐브는 손과 카메라 밖에 둔다.

```powershell
& "'''+str(command)+'''"
```

1. 터미널에서 첫 SPACE를 누른다. 빈손으로 준비 자세를 잡는다. 현재 검토한 자세 기준 빈손 준비는 약1분30초다.
2. PLACE CUBE가 뜬 뒤에만 큐브를 놓는다. 초록 면이 위, 주황 면이 손목 ID41 쪽이다. 아래 성공 사진의 놓는 위치를 참고한다. 손 없이 놓여 있어야 하며 끼워 넣지 않는다.
3. 손을 뗀 뒤 터미널에서 두 번째 SPACE를 누른다. 뒤집기와 다음 파지 자세로의 느린 이동을 촬영한다. 수평 회전은 이어서 실행하지 않는다.
4. REMOVE CUBE가 뜨면 큐브를 빼고 세 번째 SPACE로 종료한다. Q/Esc/Ctrl+C는 중단이다.

동작 자체는 약4분이며 사진과 로그는 자동 저장된다. 끝나면 ‘했어’만 알려주면 된다.

놓는 위치 참고 사진: [이전 실물 성공 시작 사진]('''+(ROOT/'loaded_thumb_rotation_20261026_135418_442474/cube_before_turn.jpg').as_posix()+''')

이번 경로는 원래 성공한 뒤집기에서 검지6번을-8틱, 약지14번을+8틱 보완한 실험이다. 그 외14개 관절의 뒤집기 목표와75ms 주기는 같다. 관절 범위와450mA 및 원래PID, 전압/온도/추종 정지 조건은 유지한다. 두 준비 자세를 지나는 이번 프로그램의 출발점 대비 총 엄지 이동량 제한만1568틱으로 따로 검토했으며 기존 수평 프로그램의1300틱 제한은 바꾸지 않았다. 사전 물리 계산은 실제 큐브의 위치 등록을 인증하지 않으므로 이번 촬영으로 보완 동작과 큐브가 남는 위치를 확인한다.
'''
    (OUT/'실행안내.md').write_text(guide,encoding='utf-8')
    note='''2026-10-28 새 연결 확인 및 다음 사용자 실행 준비 완료. COM7 어댑터와16개 모터를 PING/READ로 확인했다. 21:14와21:57 모두 모터는OFF였고3회 읽기 차이는 최대1틱, 전압5.0~5.1V, 온도20도였다. 추가 모터/토크/설정 쓰기는0회다.

다음은 `leap_pitch_handoff_observe.cmd`를 한 번 실행한다. 빈손 준비→PLACE/SPACE→수직 뒤집기→다음 파지 자세까지 느린 이동→REMOVE/SPACE→OFF다. 수평 회전까지 연달아 보내지 않는다. 원래 성공한 뒤집기의 검지6번과 약지14번만 각각-8/+8틱(약-0.703/+0.703도) 보완해 간섭 여유를 만들었고 나머지14개 관절 목표와75ms 주기는 보존했다. 보완 뒤 실물 성공은 아직 확인하지 않았다.

검사 근거는 `right_hand_rotation_20261028/two_axis_connection/operator_pitch_handoff_v3_side_clearance/readiness.json`과`review.json`이다. 빈손210조건, 전체 명령 형상201조건, 손 동역학9조건, 가설 큐브 초기 위치에서의 전달 유지27조건이 통과했다. 손바닥7/엄지 끝의 기존10µm 이음새 판정은 그대로이며 힘/형상이나 다른 접촉 기준을 완화하지 않았다. 원본v1/v2의 실패 결과도 보존했다. 소프트웨어13검사와 실제 최신 읽기값의 캐시 검증도 통과했다.

기존4틱 시작/활성화 허용 범위를 그대로 활용해 새OFF값의1틱 ADC차이 때문에 매번 같은 모델을 다시 계산하지 않도록 했다. 건강 상태는 항상 새로 확인하고4틱 밖이면 새 검토가 필요하다. 새 계산은OFF에서만 최대600초이며 실제 구동은300초 제한을 유지한다. 새 복합 경로의 총 엄지 이동량 제한은1568틱(현재 실제 요구1551)으로 별도 검토했고 이전 수평 프로그램의1300 제한이나 관절 각도 범위, 구동력, 전압/온도/추종 정지 기준은 그대로다.

목표는 그대로 오른손의 원하는 방향 약90도 회전이다. 기존 수직2회와 수평 고정 약88도/OPEN 후약82도라는 관찰은 보존한다. 이번 새 동작의 큐브 유지 위치를 확인하기 전에는 뒤이은 수평 회전이나 전체 자동풀이를 완료로 표시하지 않는다.

<!-- END PITCH HANDOFF READY 20261028 -->

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig')
        if '<!-- END PITCH HANDOFF READY 20261028 -->' not in old:path.write_text(note+old,encoding='utf-8')
    print(json.dumps(dict(status=state['status'],command=str(command),tests=13,model_cases=447,devices_opened=False),ensure_ascii=False))

if __name__=='__main__':main()
