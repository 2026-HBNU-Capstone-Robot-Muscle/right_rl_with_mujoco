"""Record v3 power stops, publish source-bound trimmed recovery v4."""
import datetime as dt,hashlib,json
from pathlib import Path
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,LOADED_PATH,HorizontalQuarterPlan,hashes,ENTRY,ROOTS,OPEN
from horizontal_quarter_gate_20261027 import validate_review
from horizontal_empty_recovery_20261027 import POLICY

def main():
    plan=HorizontalQuarterPlan(REQUEST);proof=validate_review(plan,PROOF);verification=json.loads((OUT/'verification.json').read_bytes())
    if verification['status']!='all_device_free_regressions_passed' or verification['tests_passed']!=62 or verification['code_hashes']!=hashes():raise RuntimeError('Incomplete current checks')
    previous=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v3';state=json.loads((previous/'readiness.json').read_bytes())
    digest=hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest();assert digest==state['loaded_path_sha256']
    failures=[]
    for name in ('horizontal_quarter_trial_20261027_152511_177363','horizontal_quarter_trial_20261027_152632_508556'):
        r=json.loads((ROOT/name/'result.json').read_bytes());samples=r['samples'];last=samples[-1];camera=r['camera_recording']
        assert r['all_torque_off_verified'] and r['settings_restored'] and not r.get('goal_readback_events')
        assert set(s['phase'] for s in samples)<={ENTRY,ROOTS} and r['current_start_screen_accepted']
        failures.append(dict(folder=name,error=r['error'],phase=ROOTS,motion_started=True,cube_placed=False,
            goal_readback_events=0,voltage_events=r['voltage_events'],
            minimum_voltage_v=min(v for s in samples for v in s['voltage_v']),maximum_index_root_current_ma=max(abs(s['current_ma'][7]) for s in samples),
            final_index_root=dict(goal=last['goal'][7],actual=last['actual'][7],error_ticks=last['goal'][7]-last['actual'][7]),
            all_torque_off_verified=True,settings_restored=True,usable_frames=camera['image_signal_usable_frames'],blank_frames=camera['image_signal_blank_frames']))
    rows=plan.plan(plan.source_position());roots=[q for phase,q in rows if phase==ROOTS];opened=[q for phase,q in rows if phase==OPEN][-1]
    review=dict(status='v3_power_stops_and_v4_trimmed_recovery_review',operator_stops=failures,
        cause_of_stop='both sustained undervoltage during EMPTY index-root recovery; not a camera, model admission, or goal readback failure',
        mechanical_contact_proven=False,exact_power_drop_location_measured=False,
        old_pre_open_roots=[2150,2150,2053],new_pre_open_roots=roots[-1][[7,11,15]].tolist(),
        final_index_root_open_goal=int(opened[7]),old_index_root_excursion_above_open_ticks=102,
        root_policy=POLICY,open_and_loaded_endpoints_unchanged=True,loaded_path_sha256=digest,
        tests_passed=62,current_empty_model_cases=210,loaded_geometry_cases=201,loaded_physics_cases=27,
        voltage_health_policy_changed=False,hardware_limits_changed=False,
        official_voltage_reference=dict(url='https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/',minimum_v=3.7,maximum_v=6.0,recommended_v=5.0),
        physical_voltage_improvement_verified=False,physical_horizontal_90_verified=False,
        assistant_motor_writes=0,assistant_torque_or_setting_writes=0)
    (OUT/'power_and_recovery_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    state.update(status='horizontal_quarter_v4_trimmed_recovery_ready',timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat(),
        operator_model_review=PROOF.relative_to(ROOT).as_posix(),verification=(OUT/'verification.json').relative_to(ROOT).as_posix(),
        latest_readonly='repeat_off_readonly_20261027_153205_742624/result.json',latest_readonly_stable_drift_ticks=1,
        latest_user_trial=failures[-1]['folder'],recent_operator_stops=failures,empty_root_motion_policy=dict(POLICY),
        device_free_tests_passed=62,current_empty_model_cases=210,loaded_motion_data_unchanged=True,
        nominal_active_seconds=len(rows)*plan.SAMPLE_PERIOD,next_user_action='operator_starts_v4_once',
        next_user_task='Run once with v4 header. EMPTY first SPACE; PLACE GREEN up ORANGE toward wrist ID41, hands clear then second SPACE; automatic OPEN, REMOVE then third SPACE. Send done only.',
        next_user_command='leap_horizontal_quarter_trial.cmd',next_user_command_ready=True,next_motion_command_ready=True,rotation_execution_ready=True,
        physical_voltage_improvement_verified=False,physical_horizontal_90_verified=False,horizontal_quarter_turn_verified=False,
        if_same_undervoltage_repeats='Do not repeat the same run or keep lowering stops; inspect the power delivery/contact path and saved root observations.',
        assistant_motor_writes=0,assistant_torque_or_setting_writes=0,pending_user_input=None)
    # Replace the current model pointer without losing the previous source-bound evidence.
    state['simulation_reports'][0]=dict(path=(OUT/'empty_recovery_screen/review.json').relative_to(ROOT).as_posix(),cases=210,accepted_cases=210,hardware_execution_allowed=False)
    (OUT/'readiness.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    command='& "'+str(ROOT/'leap_horizontal_quarter_trial.cmd')+'"'
    guide=f'''# 수평 회전 v4

15:25와15:26 두 실행은 모델/시작 검사와 목표 읽기를 통과했다. 빈손 검지 뿌리를 올리던 중3.6V가 검출되고150ms 안에 전체 ID가3.8V 이상으로2회 회복하지 못해 중단했다. ID8의 마지막 목표/실제는2131/2033틱과2129/2034틱이었다. 마지막 전류는175/171mA였다. 두 번 모두 OFF와 설정 복구가 확인됐다. 큐브 배치나 회전은 시작하지 않았다. 정상 영상545/530장, 검은 프레임0장이며 저장 사진에서 빈손 이동을 확인했다. 이 사진과 로그만으로 충돌 또는 정확한 전원 손실 지점은 확정하지 않는다.

기존 경로는 최종 배치 자세에서2048틱인 검지 뿌리를 미리2150틱까지 올렸다가 돌아왔다. v4는 먼저 받침 분리를 위한1980틱까지 이동하고, 검지/중지의 남은 사전 이동 목표를2064틱으로 줄인다. 이미 더 높은 출발값을 강제로 낮추지 않는다. 약지 목표2053틱은 같다. 이후 같은 OPEN 자세로2틱 이하의 새 보간을 만들므로 이전2150틱 경로로 뛰지 않는다. 최종 배치와 파지/회전/자동 열기 데이터는 동일하다.

무장치62개 검사와 현재 자세의201개 기하/9개 물리 조건이 통과했다. 기존 loaded 기하201개와 물리27개 검토를 동일 해시로 연결했다. 원래450mA/PID, 전압/전류/추종/온도 중단과 관절 범위를 유지했다. 전기적 공급 경로를 식별한 모델은 아니므로 실물 전압 개선과 수평90도는 확인 전이다. 동일 저전압이 반복되면 반복 실행이나 기준 완화 대신 전원 전달 경로를 확인한다. 공식 XC330-M288 입력 전압은3.7~6.0V, 권장5V다: https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/

```powershell
{command}
```

헤더 v4로 한 번 실행한다. 손 모양은 다시 맞추지 않는다. 큐브는 손과 카메라 밖에 두고 첫 SPACE. PLACE 뒤 초록 위/주황 손목ID41 쪽으로 배치하고 손을 뺀 뒤 두 번째 SPACE. 자동 열기 뒤 큐브를 빼고 세 번째 SPACE로 OFF한다. 끝나거나 중단되면 했어만 보내면 저장 결과를 확인한다.
'''
    (OUT/'operator_guide.md').write_text(guide,encoding='utf-8')
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json','right_hand_rotation_20261026/fresh_empty_start_20261026_235258/progress_20261027.json'):
        path=ROOT/name;data=json.loads(path.read_bytes());data.update(state);path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block='''## CURRENT 2026-10-27: 수평 v4 뿌리 과도한 사전 이동 축소

- 사용자 v3 두 실행 `horizontal_quarter_trial_20261027_152511_177363`/`...152632_508556`을 확인했다. 모두 빈손 ROOTS 구간에서3.6V 뒤 회복 조건을 못 채워 중단했다. 모델/시작/목표 읽기 오류는 없고 OFF/설정 복구가 확인됐다. 큐브 배치와 회전 전이며 정상 영상545/530장, 검은 프레임0장이다. 저장 사진은 빈손 이동과 본체 쪽 사람 손을 보여준다. 접촉이나 전원 손실 지점을 확정하지 않는다.
- ID8은 목표2131/실제2033틱, 목표2129/실제2034틱에서 각각175/171mA였다. 기존 사전 목표2150은 최종 OPEN의2048보다102틱(약9도) 높았다. v4는 받침 분리1980틱은 유지하고 ID8/12의 사전 목표를2064틱으로 줄인다. ID16 목표2053틱은 같다. 낮아진 출발점에서 OPEN 보간을 다시 만들고 실제 뿌리 분리 확인 게이트와2틱 상한을 유지한다. OPEN 및 loaded 파지/회전/자동 열기 데이터는 동일하다.
- 새15:32 PING/READ3회는 편차1틱, 최소4.5V, 최고24도였고 모터 쓰기는0회다. 새 복귀201개 기하/9개 유한 강성 물리 조건과 기존 검사를 포함한 무장치62개 검사가 통과했다. 원본 구동/관절 범위/족자 파일은 유지했다. 전압 기준과450mA/PID 등 건강 중단도 같다. 전압 개선과 실물 수평90도는 미검증이며 기존 수직 성공 총2회는 유지한다.
- 전압3.6V는 공식 XC330-M288 입력 범위3.7~6.0V보다 낮다. 공식 자료는 `https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/`다. 모델은 전원 케이블/접점 저항을 식별한 것이 아니다. v4에서도 같은 전압 중단이 반복되면 같은 실험 반복이나 소프트웨어 한도 완화 대신 전원 전달 경로 확인을 우선한다. 단일3핀 전원 구성은 사용자 선호에 따라 이번에 바꾸지 않았다.
- 다음 사용자 작업은 같은 `leap_horizontal_quarter_trial.cmd` v4를1회 실행하는 것이다. 빈손 첫 SPACE, PLACE 뒤 초록 위/주황 손목ID41 쪽 배치와 손 뺀 뒤 두 번째 SPACE, 자동 열기 뒤 큐브 제거와 세 번째 SPACE. 끝나거나 중단되면 했어만 받는다. 새 치수/촬영/로그 복사나 수동 자세 복원을 요구하지 않는다.
- 근거: `right_hand_rotation_20261027/horizontal_quarter_trial_v4/`의 `review.json`, `verification.json`, `power_and_recovery_review.json`, `readiness.json`, `operator_guide.md`. Assistant의 실제 목표/토크/설정 쓰기는0회다.

<!-- END HORIZONTAL V4 TRIMMED RECOVERY 20261027 -->

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8-sig')
        if '<!-- END HORIZONTAL V4 TRIMMED RECOVERY 20261027 -->' in text:raise ValueError('Already recorded')
        path.write_text(block+text.replace('## CURRENT 2026-10-27:','## PREVIOUS 2026-10-27:',1),encoding='utf-8')
    print(json.dumps(dict(status=state['status'],tests_passed=62,fresh_model_cases=210,loaded_data_unchanged=True,next_user_command=state['next_user_command'],assistant_motor_writes=0)))
if __name__=='__main__':main()
