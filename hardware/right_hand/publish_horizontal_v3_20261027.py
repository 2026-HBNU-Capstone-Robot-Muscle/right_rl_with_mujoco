"""Publish reviewed v3 and the actual two ROOTS failures; never opens devices."""
import datetime as dt,hashlib,json
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,LOADED_PATH,LOADED_REVIEW,STATIC_REVIEW,HorizontalQuarterPlan,hashes,ENTRY,ROOTS
from horizontal_quarter_gate_20261027 import validate_review
from horizontal_goal_confirmation_20261027 import POLICY as GOAL_POLICY
from horizontal_empty_recovery_20261027 import POLICY as ROOT_POLICY

def main():
    plan=HorizontalQuarterPlan(REQUEST);proof=validate_review(plan,PROOF);tests=json.loads((OUT/'verification.json').read_bytes())
    if tests['status']!='all_device_free_regressions_passed' or tests['code_hashes']!=hashes() or tests['skipped'] or tests['tests_passed']!=61:raise RuntimeError('Missing complete current verification')
    failures=[]
    for name in ('horizontal_quarter_trial_20261027_143018_624423','horizontal_quarter_trial_20261027_143147_188487'):
        r=json.loads((ROOT/name/'result.json').read_bytes());samples=r['samples']
        assert r['all_torque_off_verified'] and r['settings_restored'] and r['off_preload_confirmation']['accepted']
        assert set(row['phase'] for row in samples)<={ENTRY,ROOTS}
        failures.append(dict(folder=name,error=r['error'],motion_started=True,stage='EMPTY roots',cube_placed=False,
            minimum_voltage_v=min(v for row in samples for v in row['voltage_v']),
            maximum_tracking_rad=max(v for row in samples for v in row['tracking_rad']),
            all_torque_off_verified=True,settings_restored=True))
    digest=hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest()
    old=json.loads((ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v2/readiness.json').read_bytes())
    assert old['loaded_path_sha256']==digest
    rows=plan.plan(plan.source_position());stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
    state=dict(old);state.update(status='horizontal_quarter_v3_serial_roots_and_read_confirmation_ready',timestamp=stamp,
        operator_model_review=PROOF.relative_to(ROOT).as_posix(),verification=(OUT/'verification.json').relative_to(ROOT).as_posix(),
        latest_readonly='repeat_off_readonly_20261027_143741_640932/result.json',latest_readonly_stable_drift_ticks=0,
        latest_user_trial=failures[-1]['folder'],recent_operator_stops=failures,
        goal_readback_confirmation_policy=dict(GOAL_POLICY),empty_root_motion_policy=dict(ROOT_POLICY),
        device_free_tests_passed=61,current_empty_model_cases=210,loaded_static_cases=201,loaded_physics_cases=27,
        motion_controls_identical_to_v1=False,empty_recovery_queue_changed=True,loaded_motion_data_unchanged=True,
        loaded_path_sha256=digest,voltage_health_body_unchanged=True,physical_voltage_improvement_verified=False,
        goal_confirmation_real_packet_recovery_verified=False,nominal_active_seconds=len(rows)*plan.SAMPLE_PERIOD,
        next_user_command='leap_horizontal_quarter_trial.cmd',next_user_command_ready=True,
        next_user_task='Run once with v3 header. EMPTY first SPACE; PLACE GREEN up ORANGE toward wrist ID41 and hands clear before second SPACE; automatic OPEN then REMOVE before third SPACE. Send done only.',
        next_user_action='operator_starts_v3_once',next_motion_command_ready=True,rotation_execution_ready=True,
        physical_horizontal_90_verified=False,horizontal_quarter_turn_verified=False,assistant_motor_writes=0,
        assistant_torque_or_setting_writes=0,pending_user_input=None)
    state['historical_simulation_reports']=state.get('historical_simulation_reports',state.get('simulation_reports',[]))
    state['simulation_reports']=[dict(path=(OUT/'empty_recovery_screen/review.json').relative_to(ROOT).as_posix(),cases=210,accepted_cases=210,hardware_execution_allowed=False),dict(path=STATIC_REVIEW.relative_to(ROOT).as_posix(),cases=201,accepted_cases=201,hardware_execution_allowed=False),dict(path=LOADED_REVIEW.relative_to(ROOT).as_posix(),cases=27,accepted_cases=27,acceptance='safety_retention_and_ranges; not 90deg in all cases',hardware_execution_allowed=False)]
    (OUT/'readiness.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (OUT/'root_and_read_fix_review.json').write_text(json.dumps(dict(failures=failures,goal_policy=GOAL_POLICY,root_policy=ROOT_POLICY,
        interpretation='Observed ID16 previous value among 15 current values; no recovery reads were attempted in v2. Persistent power dip is real in saved telemetry, not proved to be a wiring defect.',
        fix='READ-only exact-current confirmation plus short support clearance followed by serial roots',
        rejected_model_candidates=['empty_recovery_screen_serial_only_rejected','empty_recovery_screen_clearance32_rejected','empty_recovery_screen_clearance64_rejected'],
        rejection_reason='initial support clearance exceeded existing 70-goal deadline; no criterion was relaxed',
        tests_passed=61,fresh_empty_cases_passed=210,unchanged_loaded_geometry_cases=201,unchanged_loaded_physics_cases=27,
        loaded_path_sha256=digest,physical_voltage_improvement_verified=False,physical_horizontal_90_verified=False,assistant_motor_writes=0),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    command='& "'+str(ROOT/'leap_horizontal_quarter_trial.cmd')+'"'
    guide=f'''# 수평 회전 v3

14:30 실행은 빈손 뿌리 복귀 중 ID16 목표만2033틱으로 읽혔다. 명령은2034틱이고 다른15개는 현재 명령과 같았다. 전체 벡터가 이전 패킷과 같지 않다는 이유로 추가 확인 없이 중단했다. 14:31 실행은 같은 구간에서3.6V가 검출됐다. 첫 회복은80.8ms에 성공했지만 다음에는6회 읽어도 최소3.7V여서 중단했다. 두 실행 모두 OFF와 설정 복구가 확인됐으며 PLACE와 큐브 회전에는 도달하지 않았다.

v3는 모터별 값이 현재 또는 바로 직전 명령과 정확히 같은 경우에만 목표를 고정하고 새 읽기를 한다.150ms/최대3회 안에 전체 현재 목표가 정확히 일치하고 건강한 읽기2회가 연속돼야 진행한다. 값을 허용 오차로 간주하지 않고 목표를 재전송하지 않는다. 알 수 없는 값, 미회복, 반복 오류와 다른 건강 오류는 계속 중단한다.

빈손 복귀는 먼저 각 뿌리를1980틱까지 함께 올려 기존 시간 안에 받침에서 분리한다. 이후 검지ID8, 중지ID12, 약지ID16의 남은 큰 이동을 하나씩 진행한다.32/64틱의 더 짧은 초기 후보는 받침 이탈이 늦어 거부했고 그 검토를 보존했다. 전압 기준,450mA/PID, 추종/온도/전류 한도, 관절 범위와 목표2틱 한도는 유지했다. 파지와 회전 및 자동 열기 데이터는 동일하다.

무장치61개 검사와 새 복귀201개 기하/9개 물리 조건이 통과했다. loaded 경로의 기하201개와 물리27개 검토는 동일 해시로 연결돼 있다. 전압 개선은 실물 확인 전이다. 모델 명목 회전87.28도와 조건별59.44~91.08도는 실물90도 성공을 뜻하지 않는다.

```powershell
{command}
```

헤더 v3를 확인하고 한 번 실행한다. 손 모양을 수동으로 되돌리지 않는다. 큐브를 손과 카메라 밖에 두고 첫 SPACE. PLACE 뒤 초록 위, 주황 손목ID41 쪽으로 배치하고 사람 손을 뺀 뒤 두 번째 SPACE. 자동 열기가 끝나면 큐브를 빼고 세 번째 SPACE로 OFF한다. Q/Esc/Ctrl+C는 중단이다. 완료하거나 중단되면 했어만 보내면 저장 파일을 확인한다.
'''
    (OUT/'operator_guide.md').write_text(guide,encoding='utf-8')
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261026/fresh_empty_start_20261026_235258/progress_20261027.json'):
        path=ROOT/name;data=json.loads(path.read_bytes());data.update(state);path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block='''## CURRENT 2026-10-27: 수평 v3 복귀 부하 분산 및 목표 읽기 확인

- 최근14:30/14:31 두 실행 모두 OFF preload v2는 통과하고 빈손 뿌리 이동까지 진행했다. 첫 실행은 ID16만2033틱(현재 명령2034틱, 다른15개는 현재 명령)으로 읽혔고 전체가 직전 패킷인 경우만 확인하던 분류에서 즉시 중단했다. 둘째는3.6V 뒤 첫 회복은80.8ms에 성공했지만 다음6회 확인은 최소3.7V여서 실패했다. 새 카메라/시작 검사 문제가 아니다. 두 실행 모두 OFF/설정 복구가 확인됐고 큐브 배치/파지/회전 전이다.
- 수평 프로그램 전용 읽기 확인은 각 모터가 정확한 현재/바로 직전 명령 중 하나인 경우에만 추가 READ를 한다.150ms/최대3회 안에 전체 현재 명령과 정확히 같고 건강한 읽기2회가 연속돼야 다음 목표를 보낸다. 오차 허용이나 목표 재전송은 없으며 미완성 쓰기, 알 수 없는 값, 반복 오류와 다른 건강 오류는 거부한다. 기존 공용 프로그램은 변경하지 않았다.
- 복귀는 먼저 뿌리를1980틱까지 함께 올려 받침에서 떼고 이후 ID8→12→16 순서로 남은 큰 이동을 하나씩 보낸다. 순차 이동만 하거나32/64틱만 먼저 떼는 후보는 기존 분리 시간 기준에 실패해 보존/거부했다. 최종 현재각201개 기하와9개 유한 강성 물리 조건은 모두 통과했다. 기존 전압 기준,450mA/PID,0.18rad 실제 추종 중단,2틱 목표 한도와 관절 범위는 유지한다. 실제 전압 부담 감소는 확인 전이다.
- 14:37 PING/READ3회는 편차0틱, 최소4.5V, 최고24도였고 모터 쓰기는0회다. 무장치61개 검사(이번 부분 패킷 재현, 지속 저전압의 거부, 기존 preload/활성화/정리/증거 게이트)가 통과했다. 파지/회전/자동 열기 데이터와 loaded 기하201/물리27 검토는 해시가 같으며 실물 수평90도는 여전히 미검증이다. 기존 수직 성공 총2회와 원본 동작/관절 범위/족자 파일은 유지한다.
- 다음 사용자 작업: 같은 `leap_horizontal_quarter_trial.cmd`를1회 실행하고 v3 헤더를 확인한다. 빈손 첫 SPACE, PLACE 뒤 초록 위/주황 손목ID41 쪽 배치와 손 뺀 뒤 두 번째 SPACE, 자동 열기 뒤 큐브 제거와 세 번째 SPACE다. 끝나거나 중단되면 했어만 받는다. 사용자에게 새 치수나 촬영, 로그 복사, 수동 손 모양 복원을 요구하지 않는다.
- 근거: `right_hand_rotation_20261027/horizontal_quarter_trial_v3/`의 `review.json`, `verification.json`, `readiness.json`, `root_and_read_fix_review.json`, `operator_guide.md`. Assistant의 실제 목표/토크/설정 쓰기는0회다.

<!-- END HORIZONTAL V3 ROOT AND READ FIX 20261027 -->

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8-sig')
        if '<!-- END HORIZONTAL V3 ROOT AND READ FIX 20261027 -->' in text:raise ValueError('Already recorded')
        path.write_text(block+text.replace('## CURRENT 2026-10-27:','## PREVIOUS 2026-10-27:',1),encoding='utf-8')
    print(json.dumps(dict(status=state['status'],tests_passed=61,current_empty_cases_passed=210,nominal_active_seconds=state['nominal_active_seconds'],loaded_path_unchanged=True,next_user_command=state['next_user_command'],assistant_motor_writes=0),ensure_ascii=False))
if __name__=='__main__':main()
