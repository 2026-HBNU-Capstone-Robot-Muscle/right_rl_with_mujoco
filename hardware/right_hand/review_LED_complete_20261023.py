"""Review the operator's completed OFF test and existing motion readiness."""
import bisect
import hashlib
import json
from pathlib import Path
import numpy as np
from grasped_yaw_plan import GraspedYawPlan
from thumb_live_start_gate import validate_proof

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261023'


def main():
    candidates=[]
    for path in sorted(ROOT.glob('bus_echo_off_20261023_*/result.json')):
        r=json.loads(path.read_bytes())
        if r.get('diagnostic_revision')==2:candidates.append((path,r))
    source,r=candidates[-1];blob=source.read_bytes();rows=r['reads'];writes=r['led_writes']
    assert r['status']=='OFF_LED_echo_complete'
    assert r['all_torque_off_verified'] and r['led_restoration_verified']
    assert all(r[key]==0 for key in ('goal_writes','torque_writes','drive_setting_writes','eeprom_writes'))
    assert not r.get('read_failures') and not r.get('echo_mismatches')
    assert r['camera_worker_stopped'] and all(p['status']=='complete' for p in r['phases'])
    assert all(not any(row[key]) for row in rows for key in ('torque','error','watchdog'))
    stamps=[row['t'] for row in rows];ack_latency=[]
    for w in writes:
        assert w['address']==65 and w['size']==1 and w['tx_returned_success']
        after=bisect.bisect_right(stamps,w['t'])
        assert after<len(rows) and rows[after]['led']==w['led']
        ack_latency.append((rows[after]['t']-w['t'])*1000)
    assert rows[-1]['led']==r['original_led_values']
    latest=np.asarray(rows[-1]['position'])
    matches=[];previous_differences=[]
    for label in ('grasped_yaw_fixture','grasped_yaw_prior_fixture','grasped_yaw_open_fixture',
                  'grasped_yaw_after_stop_fixture','grasped_yaw_goal_stop_fixture',
                  'grasped_yaw_LED_complete_fixture'):
        folder=OUT/label;plan=GraspedYawPlan(folder/'fresh_off_request.json')
        proof=validate_proof(plan,folder/'native_screen.json')
        difference=latest-plan.source_position()
        previous_differences.append(dict(fixture=label,max_difference_ticks=int(max(abs(difference)))))
        if max(abs(difference))<=4:
            plan.validate_start(latest)
            path=plan.plan(latest)
            matches.append(dict(fixture=label,maximum_start_difference_ticks=int(max(abs(difference))),
                path_goal_count=len(path),static_cases=len(proof['static_screens']),
                dynamic_cases=len(proof['dynamic_sensitivity'])))
    assert matches,'Latest saved OFF angles need a separate geometry review'
    validation=json.loads((OUT/'grasped_yaw_validation.json').read_bytes())
    assert validation['passed'] and validation['tests_run']==249
    integration=json.loads((OUT/'ring_order_integration_validation.json').read_bytes())
    assert integration['passed'] and len(integration['fixtures'])==6
    review=dict(status='OFF_LED_comparison_complete_next_small_yaw_operator_trial',
        source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(blob).hexdigest(),
        phases=r['phases'],health_snapshots=len(rows),LED_probe_packets=sum(x['probes'] for x in r['phases']),
        LED_restore_packets=sum(w['purpose']=='restore' for w in writes),
        read_failures=0,LED_mismatches=0,all_LED_writes_followed_by_exact_fresh_read=True,
        read_ms_percentiles_0_50_95_100=np.percentile([x['read_ms'] for x in rows],[0,50,95,100]).tolist(),
        LED_ack_ms_percentiles_0_50_95_100=np.percentile(ack_latency,[0,50,95,100]).tolist(),
        minimum_voltage_v=min(min(x['voltage']) for x in rows)/10,
        maximum_temperature_c=max(max(x['temperature']) for x in rows),
        maximum_saved_OFF_position_range_ticks=int(np.max(np.ptp([x['position'] for x in rows],axis=0))),
        all_torque_off_verified=True,original_LED_values_restored=True,camera_worker_stopped=True,
        goal_torque_drive_eeprom_writes=0,saved_start_neighborhood_matches=matches,
        start_differences_to_saved_fixtures=previous_differences,
        loaded_four_byte_goal_delivery_verified=False,physical_communication_root_cause_resolved=False,
        actual_cube_rotation_verified=False,live_SAC=False,
        next_trial_uses_existing_read_recovery_confirmation=True,
        motion_program_changed=True,motion_health_or_native_limits_changed=False,
        empty_ring_order_changed=True,loaded_grasp_and_yaw_waypoints_changed=False,
        motion_regression_tests=249,motion_regression_tests_rerun=True,
        native_current_and_prior_starts=integration['fixtures'],
        next_execution_requires_new_stable_OFF_readings_and_native_path_screen=True,
        next_user_command='leap_grasped_yaw_trial.cmd',additional_LED_or_calibration_capture_required=False,
        raw_execution_results_modified=False,assistant_opened_real_devices=False)
    target=OUT/'LED_complete_review.json'
    target.write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8')
    title='## CURRENT 2026-10-23 23:31: LED 통신 검사 완료 / 빈손 약지 준비 순서 수정'
    note='''

- 최신 실행 `bus_echo_off_20261023_233131_294853`가 정상 완료됐다. 카메라를 끈 조건에서 LED 시험297개, 카메라를 켠 조건에서296개가 확인됐으며 전체 건강 상태600회를 읽었다. 읽기 오류와 LED 불일치는 없었다. 기록된 LED 쓰기594개는 모두 그 뒤의 첫 새 읽기에서 정확한 값이 확인됐다. 이 중1개는 원래 LED 값 복원이다.
- 모든 읽기에서 토크와 오류 및 watchdog은0이다. 마지막 원래 LED 값 복원과 웹캠 프로세스 종료를 확인했다. 목표와 토크, 전류, PID, 모드 및 EEPROM 쓰기는0회다. 최소 전압4.3V, 최장 전체 상태 읽기32ms이며 관절별 저장 OFF 위치 범위는 최대1틱이다. 카메라 조건에서295프레임을 수신했고274개 새 패킷의 시간 범위는 약14.922초다.
- 이 결과는 이번 토크 OFF 조건에서 카메라 사용과 LED 통신을 함께 수행할 수 있음을 보여준다. 모터당1바이트 LED 쓰기이므로 부하가 걸린4바이트 목표 위치 전송까지 정상임을 입증하지 않는다. 앞선 토크 ON 파지 중의 목표 미반영 원인이 해결됐다고 주장하지 않는다.
- 최신 저장 자세로 기존 준비 순서를 검사하니 빈손을 여는 도중 약지 끝과 엄지 끝의 모델 간섭이 최대 약0.704mm 발생해 경로 검사가 거부됐다. 실제 접촉을 측정한 값은 아니다. 이전 토크 ON 파지 중단 뒤 저장된 자세와는 검지 PIP341틱 및 약지 PIP154틱 등의 차이가 있었지만 LED 검사30초 동안의 관절 범위는 최대1틱이다. 변한 원인을 추정으로 확정하지 않는다.
- 작은 회전 시험의 빈손 준비 순서를 수정했다. 기존 약지와 손바닥의 짧은 분리 경로 및 사용자 분리 확인을 보존한 뒤, 약지 끝을 펴는 분리가 필요했던 시작에서는 약지 네 관절을 도달한 기존 준비 위치로 먼저 옮긴다. 이어 엄지 및 다른 손가락을 준비한다. 나머지 시작은 원래 경로에 짧은 고정 목표 확인 구간을 더한다. 큐브를 놓은 뒤 파지와 회전 목표 목록, 전류와 PID, 목표 확인, 전압 및 추종 중단 기준과 관절 범위는 바꾸지 않았다.
- 현재 저장 자세와 다섯 기존 시작 자세에서 각각67개 정적 및3개 응답 검사를 통과했다. 현재 자세의 새 경로에서는 허용하지 않은 접촉이0이고 최대 모델 추종 오차는 약0.06062rad이다. 장치 없는 회귀 검사249개도 통과했다. 원래 실패 증명은 별도 이름으로 보존했다. 저장 자세의 증명은 현재 실물 시작 승인으로 쓰지 않으며 다음 실행에서는 새 안정 OFF 읽기와 경로 검사를 다시 수행한다.
- 다음 사용자 실행은 `leap_grasped_yaw_trial.cmd`다. 빈손 약지 준비 순서 수정 및 읽기 오류 복구 후 같은 목표의 건강 상태를 두 번 더 확인하는 기존 수정이 적용돼 있다. 자동 동작 재시도나 목표 재전송 없이 한 번만 수행한다. 파지 후 모델상 작은 수평 회전과 제거 전 관찰을 기록한다. 실제 목표 방향 회전과 실물 SAC 정책은 여전히 미검증이다.
- **지금 사용자 작업:** LED 검사 창을 닫고 큐브를 손과 카메라 화면 밖으로 뺀다. 같은 손 자세에서 `leap_grasped_yaw_trial.cmd`를 실행한다. 빈손 첫 SPACE, 약지 분리 확인 SPACE, PLACE에서 초록 위와 주황 카메라 쪽으로 놓고 손을 뺀 뒤 SPACE다. REMOVE가 나오기 전까지 손이나 큐브를 만지지 않는다. REMOVE에서 큐브를 부드럽게 빼고 SPACE로 끝낸다. 걸리면 Q로 OFF한 뒤 제거한다. 끝나면 했어만 알려주면 된다. 추가 LED 검사나 마커 촬영은 요청하지 않는다.
- 근거: `right_hand_rotation_20261023/LED_complete_review.json`. 원본 실물 결과와 기존 네이티브 증명은 수정하지 않았다.

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith(title):continue
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(title+note+old,encoding='utf-8')
    print(json.dumps(review,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
