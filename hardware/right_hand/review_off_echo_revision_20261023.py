"""Review saved OFF evidence and test v2 without opening any real device."""
import hashlib
import io
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261023'


def main():
    import test_bus_echo_off
    capture=io.StringIO()
    result=unittest.TextTestRunner(stream=capture,verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromModule(test_bus_echo_off))
    validation=dict(tests_run=result.testsRun,passed=result.wasSuccessful(),
        real_serial_or_camera_opened=False,output=capture.getvalue())
    (OUT/'off_led_echo_validation.json').write_text(json.dumps(validation,indent=2),encoding='utf-8')
    print(capture.getvalue())
    if not result.wasSuccessful():raise RuntimeError('LED diagnostic tests failed')
    evidence=[]
    for name in ('bus_echo_off_20261023_225756_510581','bus_echo_off_20261023_225847_477065'):
        file=ROOT/name/'result.json';raw=file.read_bytes();r=json.loads(raw)
        rows=r['reads']
        assert r['all_torque_off_verified'] and r['goal_restoration_verified']
        assert all(row['goal']==row['position'] for row in rows)
        evidence.append(dict(source=str(file),sha256=hashlib.sha256(raw).hexdigest(),
            status=r['status'],error=r['error'],reads=len(rows),
            goal_equals_position_reads=sum(row['goal']==row['position'] for row in rows),
            goal_probe_writes=sum(row['phase']=='camera_closed' for row in r.get('goal_writes',[])),
            restoration_writes=sum(row['phase']=='restore_original_goals_OFF' for row in r.get('goal_writes',[])),
            camera_opened='camera_startup' in r,all_torque_off_verified=r['all_torque_off_verified'],
            original_goal_restoration_observed=r['goal_restoration_verified']))
    assert evidence[1]['goal_probe_writes']==0
    # Check existing native fingerprints only; no need to repeat unchanged motion simulations.
    from grasped_yaw_plan import GraspedYawPlan
    from slow_grasp_demo_plan import SlowGraspDemoPlan
    from thumb_live_start_gate import validate_proof
    proofs=[]
    for label in ('grasped_yaw_fixture','grasped_yaw_prior_fixture','grasped_yaw_open_fixture',
                  'grasped_yaw_after_stop_fixture','grasped_yaw_goal_stop_fixture','slow_grasp_demo_fixture'):
        folder=OUT/label;kind=SlowGraspDemoPlan if label=='slow_grasp_demo_fixture' else GraspedYawPlan
        proof=validate_proof(kind(folder/'fresh_off_request.json'),folder/'native_screen.json')
        proofs.append(dict(fixture=label,static_cases=len(proof['static_screens']),
            dynamic_cases=len(proof['dynamic_sensitivity'])))
    review=dict(status='OFF_LED_diagnostic_ready_for_operator',saved_v1_evidence=evidence,
        diagnosis='Goal Position did not remain fixed while OFF; v1 echo assumption invalid for these observations.',
        firmware_root_cause_verified=False,physical_communication_root_cause_resolved=False,
        original_latest_motion_goal_failure_still_unresolved=True,
        latest_motion_source='grasped_yaw_trial_20261023_222816_628099',
        replacement_register=dict(name='LED',address=65,bytes=1,values=[0,1],memory='RAM'),
        official_source='https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#led65',
        goal_torque_drive_eeprom_writes=0,restore_original_LED_values=True,
        existing_motion_paths_or_limits_changed=False,tests_run=result.testsRun,
        tests_passed=True,unchanged_motion_native_proofs_validated=proofs,
        raw_execution_results_modified=False,assistant_opened_real_devices=False,
        actual_cube_rotation_verified=False,loaded_goal_delivery_verified=False,
        limitation='One-byte LED writes with 83-byte reads compare camera traffic conditions; they do not prove loaded four-byte Goal Position delivery.',
        next_user_command='leap_bus_echo_off.cmd',needs_markers_or_cube_placement=False,
        source_sha256=hashlib.sha256((ROOT/'bus_echo_off.py').read_bytes()).hexdigest())
    (OUT/'off_echo_revision_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8')
    title='## CURRENT 2026-10-23 22:58: OFF 진단의 목표 고정 가정 수정 / LED 통신 검사 준비'
    note='''

- 저장된 OFF 진단 두 실행을 검토했다. 22:57 실행은 목표 시험 패킷 한 번 뒤 중단됐고, 22:58 실행은 시험 패킷 전 비교에서 중단됐다. 두 실행의 총14개 읽기에서 목표 위치116과 현재 위치132가 매번 같았고 일부 관절에서 1틱씩 달라졌다. 카메라 단계는 어느 실행도 시작하지 않았다. 마지막 모든 토크 OFF와 당시 원래 목표 값 관찰은 확인됐다.
- 이 결과는 이전 진단이 토크 OFF 목표 레지스터를 고정 메모리처럼 가정한 데 문제가 있음을 보여준다. 이 자료만으로 펌웨어 원인이나 통신 고장을 확정하지 않는다. SDK의 주소별 모의 원시 패킷에서도 LED65, 목표116, 현재132가 분리돼 읽히는 것을 확인했다. 22:28 토크 ON 파지 중의 목표 미반영 문제는 별도로 남아 있다.
- 같은 `leap_bus_echo_off.cmd`를 v2 LED 검사로 수정했다. 공식 제어표의 LED RAM65/1바이트만 기록하며 토크와 목표, 전류, PID, 모드 및 EEPROM은 쓰지 않는다. 15초는 카메라를 닫고, 15초는 웹캠을 자동으로 켠다. 매 쓰기 전 새 OFF 및 건강 상태를 확인하고, 끝나면 각 모터의 원래 LED 값을 확인 및 복원한다. 시험 패킷 재전송은 하지 않는다.
- 새 장치 없는 검사15개를 통과했다. 실제 저장된 OFF 목표 변동, 주소별 SDK 파싱, 카메라 전후 정상 완료, OFF 건강 상태와 시간 초과, 중간 ON 쓰기 차단, LED 전송 누락, 부분 전송 오류, Q와 카메라 오류, 오래된 프레임, 원래 LED 복원 실패를 포함한다. 회전 프로그램은 수정하지 않았으며 여섯 기존 네이티브 증명의 현재 코드 지문을 다시 확인했다. 원본 실물 결과는 수정하지 않았다.
- LED 쓰기는 모터당1바이트이므로 부하 상태의 목표 위치4바이트 쓰기까지 검증하는 것은 아니다. 카메라 사용 전후의 통신 상태를 비교하고 다음 동작 시험의 근거로 사용한다. 실제 목표 방향 회전과 실물 SAC 정책은 계속 미검증이다.
- **지금 사용자 작업:** 큐브를 빼고 다른 모터 및 카메라 프로그램을 닫는다. USB와 모터 전원을 연결한 채 같은 `leap_bus_echo_off.cmd`를 실행하고 터미널에서 SPACE 한 번 누른다. 손가락 모양을 바꾸거나 마커를 보여줄 필요는 없다. LED가 깜박이고 웹캠이 나중에 자동으로 켜진다. 카메라 시작 시간을 제외한 약30초 검사가 끝나면 했어만 알려주면 된다.
- 근거: `right_hand_rotation_20261023/off_echo_revision_review.json`, `off_led_echo_validation.json`. 공식 LED 설명은 https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#led65 이다.

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith(title):continue
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(title+note+old,encoding='utf-8')
    print(json.dumps({k:review[k] for k in ('status','tests_run','next_user_command')},ensure_ascii=False))


if __name__=='__main__':main()
