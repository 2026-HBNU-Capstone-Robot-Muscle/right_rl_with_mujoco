"""Record the startup watchdog diagnosis and the reviewed timing fix."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261023'

def main():
    from supported_cube_rotation_plan import SupportedCubeRotationPlan
    from thumb_live_start_gate import validate_proof
    source=ROOT/'cube_supported_rotation_trial_20261023_033108_297056'
    stopped=json.loads((source/'result.json').read_bytes())
    before=json.loads((OUT/'startup_profile_before.json').read_bytes())
    after=json.loads((OUT/'startup_profile_after.json').read_bytes())
    native=json.loads((ROOT/'right_hand_rotation_20261022/integration_validation.json').read_bytes())
    tests=json.loads((OUT/'regression_tests.json').read_bytes())
    assert stopped['all_torque_off_verified'] and stopped['settings_restored']
    assert len(stopped['samples'])==1 and set(stopped['samples'][0]['watchdog'])=={255}
    assert native['passed'] and len(native['fixtures'])==21 and tests['passed'] and tests['tests_run']==161
    for suffix in ('latest_off','prior','watchdog_stop'):
        folder=OUT/('fixture_loaded_supported_'+suffix)
        validate_proof(SupportedCubeRotationPlan(folder/'request.json'),folder/'screen.json')
    plan=SupportedCubeRotationPlan(source/'fresh_off_request.json')
    old=json.loads((source/'current_start_screen.json').read_bytes())
    assert plan.plan_digest(plan.source_position())==old['plan_sha256']
    result=dict(status='startup_watchdog_fix_ready_for_operator',
        failed_run=source.name,failed_phase=stopped['failed_phase'],
        torque_off_verified=True,settings_restored=True,
        initial_voltage_v=min(stopped['samples'][0]['voltage_v']),
        initial_peak_current_ma=max(abs(v) for v in stopped['samples'][0]['current_ma']),
        first_sample_after_enable_ms=stopped['samples'][0]['t']*1000,
        diagnosis='Repeated planning/admission JSON reads after enable starved communication; all 16 watchdogs latched 255.',
        watchdog_value_preserved=25,watchdog_interval_ms_preserved=500,
        before_post_enable_planning_ms=[s['post_enable_compute_ms'] for s in before['iterations']],
        after_post_enable_planning_ms=[s['post_enable_compute_ms'] for s in after['iterations']],
        change='Verify evidence and cache immutable endpoints, limits and goals while OFF; adopted-start validation and interpolation use memory only.',
        trajectory_digest_unchanged=True,goal_write_retries=0,maximum_goal_step_ticks=2,current_cap_ma=450,
        tests_passed=161,windows_wsl_starts_validated=21,agent_hardware_commands_sent=0,
        next_launcher='leap_cube_supported_rotation.cmd',physical_rotation_verified=False,
        official_watchdog_reference='https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#bus-watchdog98')
    (OUT/'startup_watchdog_fix_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    ready=OUT/'supported_loaded_ready_review.json'
    record=json.loads(ready.read_bytes());record.update(status=result['status'],
        latest_startup_fix_review='right_hand_rotation_20261023/startup_watchdog_fix_review.json',
        tests_passed=161,windows_wsl_starts_validated=21)
    ready.write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    section='''## CURRENT 2026-10-23 03:31 이후: 초기 워치독 정지 수정, 같은 큐브 시험 재실행 필요

- 사용자 실행 `cube_supported_rotation_trial_20261023_033108_297056`은 첫 동작 이전 hold 표본에서16개 워치독255로 중단. 첫 표본은ON 후672ms였고 최저4.5V·최대1mA·각도오차0.토크OFF/워치독초기화/설정 복원 확인. 큐브PLACE까지 도달하지 않았고 회전을 시험하지 않았다.
- 원인은조수가 추가한 계획/시작 검증이 토크ON 뒤에도 과거JSON을 반복해서 읽은 것. 실제저장요청의 순수계획 프로파일에서 adopt+두번째계산이약419~448ms였고, 다른시작처리/읽기와 합쳐서25×20ms 워치독 통신시간을 소진했다. ROBOTIS 공식 설명에서255(-1)는워치독오류이며,ON 상태의명령패킷 간격을 감시한다.
- 새 계획객체는OFF 구성/증명검증때 끝점·원래범위·후속목표를 검증하고읽기전용 배열로 보관한다. ON 후실제허용±4틱 시작의검증과보간은메모리만사용한다. 복귀/시간/범위/목표큐/배치·제거/전류450mA·75초·워치독25를유지한다. 경로SHA는실패한원본경로와동일하다. 같은순수계획측정은약12~15ms다.
- 느린파일읽기(ON중260ms 지연)와500ms 워치독을모사한 회귀검사,ON후파일접근금지검사,실제실패경로동일성검사를추가했다. 전체161개 회귀검사와저장시작21개 증명검증을통과했다. 현재코드로바뀐새시작3개는각각67정적/3응답을다시재생했고변경없는기존18개는완전한해시·목표·시작검증을거쳐증명을재사용했다. 조수는실제장치를열지않았다.
- 사용자할일:같은 `leap_cube_supported_rotation.cmd`를 큐브없이실행. SPACE시작,약지분리확인SPACE,PLACE때초록중심면위/주황중심면카메라쪽으로배치하고손뺀뒤SPACE. 작은회전후큐브제거/SPACE로OFF. 닿거나걸리면Q. 손자세를다시만들거나전원을반드시껐다킬필요는없다. 다음했어에저장결과를직접검토한다.
- 근거: `right_hand_rotation_20261023/startup_watchdog_fix_review.json`, `startup_profile_before.json`, `startup_profile_after.json`, `regression_tests.json`, `right_hand_rotation_20261022/integration_validation.json`.

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;oldtext=path.read_text(encoding='utf-8')
        if not oldtext.startswith(section):path.write_text(section+oldtext,encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('status','tests_passed','trajectory_digest_unchanged','next_launcher')},indent=2))

if __name__=='__main__':main()
