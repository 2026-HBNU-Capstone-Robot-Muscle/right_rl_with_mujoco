"""Publish measured power recovery and a reviewed bounded ring-side escape."""
import datetime as dt,hashlib,json
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,LOADED_PATH,HorizontalQuarterPlan,hashes,RING_CLEAR
from horizontal_quarter_gate_20261027 import validate_review
from rewired_initial_pair_scope_20261027 import POLICY

def main():
    plan=HorizontalQuarterPlan(REQUEST);proof=validate_review(plan,PROOF);tests=json.loads((OUT/'verification.json').read_bytes())
    if tests['tests_passed']!=74 or tests['code_hashes']!=hashes() or tests['failures'] or tests['errors'] or tests['skipped']:raise RuntimeError('Incomplete current verification')
    old=json.loads((ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v4/readiness.json').read_bytes())
    digest=hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest();assert digest==old['loaded_path_sha256']
    source=ROOT/'repeat_off_readonly_20261027_183434_733420/result.json';reads=json.loads(source.read_bytes());baud=ROOT/'baud_readonly_20261027_183436_420182/result.json';b=json.loads(baud.read_bytes())
    assert reads['status']=='off_snapshots_saved' and reads['servo_writes']==0 and reads['torque_changes']==0 and b['all16_value5_3mbps']
    photo=ROOT/'rewired_contact_readonly_20261027_184157_120144/result.json';camera=json.loads(photo.read_bytes());assert camera['stationary_endpoints'] and camera['motor_writes']==0
    delta=max(abs(a-v) for a,v in zip(plan.source_position(),camera['before']['position_ticks']))
    rows=plan.plan(plan.source_position());stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat();separation=proof['initial_self_separation']
    evidence=dict(status='power_restored_and_v5_ring_separation_ready',timestamp=stamp,
        source=source.relative_to(ROOT).as_posix(),baud_source=baud.relative_to(ROOT).as_posix(),photo_source=photo.relative_to(ROOT).as_posix(),photo_source_pose_difference_ticks=int(delta),
        motor_ids_responding=list(range(1,17)),all16_baud_register_value5=True,host_bps=3000000,
        off_voltage_v=[min(v/10 for s in reads['snapshots'] for v in s['voltage']),max(v/10 for s in reads['snapshots'] for v in s['voltage'])],
        all_torque_off=True,all_errors_zero=True,off_read_drift_ticks=reads['maximum_drift_ticks'],under_load_power_verified=False,
        physical_visible_contact_report=False,geometry_calibration_verified=False,
        exact_CAD_model_intersection_confirmed=True,convexification_alone_does_not_explain_the_disagreement=True,
        initial_scope=separation,scope_is_not_global_collision_tolerance=True,initial_depth_must_not_increase=True,
        every_other_pair_and_all_contacts_after_ring_keep_original_criteria=True,shapes_and_contact_forces_unchanged=True,
        ring_delta_ticks=96,ring_degrees=96*360/4096,ring_goals=48,required_actual_ring_ticks=separation['required_actual_ring_rotation_ticks'],
        source_initial_pair_mm=max(v['initial_pair_mm'] for v in proof['empty_recovery']['dynamic_cases']),worst_static_initial_pair_mm=separation['maximum_initial_pair_mm'],
        model_cases_passed=210,device_free_tests_passed=74,loaded_path_sha256=digest,loaded_targets_unchanged=True,
        prior_approval_usage_limit_resolved=True,analysis_python_fcl_install_completed=True,
        physical_horizontal90_verified=False,assistant_motor_writes=0,assistant_torque_or_setting_writes=0)
    (OUT/'rewired_release_review.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    state=dict(old);state.update(status=evidence['status'],timestamp=stamp,
        operator_model_review=PROOF.relative_to(ROOT).as_posix(),verification=(OUT/'verification.json').relative_to(ROOT).as_posix(),
        latest_readonly=source.relative_to(ROOT).as_posix(),latest_readonly_stable_drift_ticks=reads['maximum_drift_ticks'],latest_contact_photo=photo.relative_to(ROOT).as_posix(),
        current_baud_readonly=baud.relative_to(ROOT).as_posix(),all16_communication_verified=True,off_voltage_v=evidence['off_voltage_v'],
        pending_user_input=None,next_user_command='leap_horizontal_quarter_trial.cmd',next_user_command_ready=True,next_motion_command_ready=True,rotation_execution_ready=True,
        next_user_action='Close Wizard, user-start v5 once with empty hand, then PLACE/SPACE; REMOVE after automatic OPEN/SPACE.',
        next_user_task='Keep current fingers. EMPTY first SPACE; GREEN up ORANGE toward wrist ID41 at PLACE then second SPACE after hands clear; automatic OPEN then REMOVE before third SPACE. Send done only.',
        initial_self_separation=separation,hardware_health_thresholds_unchanged=True,model_initial_pair_classification_changed=True,
        physical_voltage_improvement_verified=False,idle_voltage_improvement_verified=True,physical_loaded_voltage_verified=False,
        motion_controls_identical_to_v1=False,loaded_motion_data_unchanged=True,nominal_active_seconds=len(rows)*plan.SAMPLE_PERIOD,
        physical_horizontal_90_verified=False,horizontal_quarter_turn_verified=False,device_free_tests_passed=74,
        assistant_motor_writes=0,assistant_torque_or_setting_writes=0,approval_usage_block_currently_resolved=True)
    state['simulation_reports'][0]=dict(path=(OUT/'empty_recovery_screen/review.json').relative_to(ROOT).as_posix(),cases=210,accepted_cases=210,initial_model_pair_scope=separation,hardware_execution_allowed=False)
    (OUT/'readiness.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    command='& "'+str(ROOT/'leap_horizontal_quarter_trial.cmd')+'"'
    guide=f'''# 배선 변경 뒤 v5

16개가3Mbps에서 응답하고 EEPROM Baud Rate값은 전부5다. 정지 전압은5.0~5.1V로 이전4.4~4.9V보다 높아졌다. 움직일 때 전압 개선은 아직 확인 전이다.18:26~18:27의 무응답은 사용자 전원 확인 뒤18:34 재읽기에서 해소됐다. 모터bps나 설정을 변경하지 않았다.

실물 사진과 사용자 관찰에는 부품 사이 틈이 있지만 CAD 모델 초기 자세에서는 중지/약지 distal 구조물 한 쌍이 약1.11mm 겹친다. 삼각형 검사에서도 모델의 교차가 확인되어 볼록 껍질 오검출만으로 설명하지 않는다. 모델의 실물 기하 보정 완료로 주장하지 않는다.

v5는 현재처럼 뿌리가 이미1980틱 이상이고 약지가 안쪽인 빈손 자세에서 약지 옆 모터ID14를96틱(약8.44도) 먼저 벌린다. 초기 모델 겹침은 지정된 한 쌍/최대2mm 안에서만, ENTRY와 첫48개 분리 목표 동안 다룬다. 정적 경로는 한 단계도 더 깊어지면 안 되고 동적 경로도 시작 깊이를 넘으면 안 된다. 다른 모든 쌍과 이후 구간의 원래 자기충돌 기준은 그대로다. 형상과 접촉력, 질량, 관절 범위는 바꾸지 않았다. 실제 ID14가2058틱 이상 읽혀야 뿌리 이동으로 진행한다.

현재201개 기하/9개 유한 강성 물리 조건과74개 무장치 검사가 통과했다. 모델상 분리 뒤 겹침은0이고 나머지 복귀에 새 겹침도 없다. 원래450mA/PID, 전압/전류/온도/0.18rad 추종 중단은 유지한다. loaded 파지/회전/자동 열기 경로와 기존 기하201/물리27 검토는 해시가 같다. 실물90도와 새 분리 동작, 부하 전압은 미검증이다.

위자드를 닫고 다음을 한 번 실행한다.

```powershell
{command}
```

손 모양은 맞추지 않는다. 빈손이며 큐브는 카메라 밖에 둔 채 첫 SPACE. 약지 분리와 준비가 끝나고 PLACE가 뜨면 초록 위/주황 손목ID41 쪽으로 큐브를 놓는다. 사람 손을 빼고 두 번째 SPACE. 회전과 자동 열기가 끝나면 큐브를 빼고 세 번째 SPACE로 OFF한다. 끝나거나 멈추면 했어만 보내면 저장 파일을 읽는다.

참고: `make_horizontal_scoped_helpers_20261027.py`는 v5 이전의 생성 기준이다. v5 checker를 덮어쓰지 않는다. 수정 시 전체 증거를 다시 검사한다.
'''
    (OUT/'operator_guide.md').write_text(guide,encoding='utf-8')
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261026/fresh_empty_start_20261026_235258/progress_20261027.json'):
        path=ROOT/name;data=json.loads(path.read_bytes());data.update(state);path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block='''## CURRENT 2026-10-27: 배선 뒤 전원 복구 확인 및 초기 약지 분리 v5 준비

- 사용자 전원 확인 뒤18:34 READ/PING에서16개 모두3Mbps 응답, 전부 토크OFF/오류0/워치독0,3회 편차1틱, 정지 전압5.0~5.1V를 확인했다. EEPROM Baud Rate(8) 값은16개 모두5(3Mbps)다.18:26~18:27 무응답은 현재 해소됐다. 위자드 연결창의bps와 모터 제어표 저장bps는 구별하며 설정 쓰기는 하지 않았다. 동작 중 전압 개선은 미검증이다.
-18:41 웹캠5장과 전후 READ 차이1틱을 보존했다. 사용자는 실제 부품이 붙어 있지 않다고 했고 사진에서도 틈이 보인다. CAD의 중지/약지 distal 한 쌍은 새 현재각에서1.1128mm(±4틱 정적 최악1.6691mm) 겹쳤다. visual/collision 변환은 같고 FCL 실제 삼각형 모델도 초기 교차를 확인했다. 볼록 껍질만의 문제로 치부하거나 실물 기하 보정 완료를 선언하지 않았다.
- 단순 뿌리 순서 변경은 초기/재접촉을 해소하지 못해 거부했다. 약지 옆 ID14를+96틱(8.44도) 먼저 벌리는 후보는67개에서 초기 깊이 증가0, 다른 쌍0, 분리 이후 겹침0이었다. 반대 방향은 깊어져 거부했다. v5는 뿌리가 이미1980틱 이상이고 약지 안쪽/MF끝이 굽은 빈손에만 이 첫 구간을 붙인다. 실제 ID14≥2058틱 확인 후에만 뿌리 목표를 보낸다.
- 초기 한 쌍에 한한 모델 분리 범위를 명시했다: 최대2mm/ENTRY와 RING48목표 이내, 정적 한 단계도 증가 금지, 동적 시작 깊이 초과 금지, 분리 후 및 다른 모든 쌍의 원래 자기충돌 기준 유지. 형상/접촉력/질량/관절 및 실제 전압/전류/추종/온도/450mA/PID는 바꾸지 않았다. 이는 전역 충돌 허용치 확대나 교정된 기하 모델이라는 주장이 아니다.
- 새 현재각201개 기하/9개 물리 조건과 무장치74개 검사가 통과했다. loaded 파지/회전/자동 열기 데이터는 동일하고 기하201/물리27 검토를 해시로 재사용한다. python-fcl 설치의 초기 사용량 검토 실패는18:16 뒤 재승인돼0.7.0.11 설치 완료로 해소됐다. 현재 승인 장애는 없다. 실제 모터/토크/설정 쓰기는0회다.
- 다음 사용자 작업: 위자드를 닫고 같은 `leap_horizontal_quarter_trial.cmd`를1회 실행해 v5를 확인한다. 빈손 첫 SPACE, PLACE 뒤 초록 위/주황 손목ID41 쪽 배치와 손 뺀 뒤 두 번째 SPACE, 자동 열기 뒤 큐브 제거와 세 번째 SPACE. 완료/중단 뒤 했어만 받는다. 새 촬영/치수/수동 자세 재구성을 요구하지 않는다.
- 근거: `right_hand_rotation_20261027/horizontal_quarter_trial_v5_rewired/`의 `review.json`, `verification.json`, `rewired_release_review.json`, `readiness.json`, `operator_guide.md`; 실제 읽기는 `repeat_off_readonly_20261027_183434_733420`, `baud_readonly_20261027_183436_420182`, 사진은 `rewired_contact_readonly_20261027_184157_120144`. 실물 수평90도는 미검증이고 수직 성공 총2회는 유지한다.

<!-- END V5 REWIRED SEPARATION READY 20261027 -->

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8-sig')
        if '<!-- END V5 REWIRED SEPARATION READY 20261027 -->' in text:raise ValueError('Already recorded')
        path.write_text(block+text.replace('## CURRENT 2026-10-27:','## PREVIOUS 2026-10-27:',1),encoding='utf-8')
    print(json.dumps(dict(status=state['status'],tests_passed=74,fresh_model_cases=210,baud_registers_all5=True,idle_voltage_v=evidence['off_voltage_v'],photo_source_difference_ticks=int(delta),next_user_command=state['next_user_command'],assistant_motor_writes=0)))
if __name__=='__main__':main()
