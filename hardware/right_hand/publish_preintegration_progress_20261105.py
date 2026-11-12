"""Publish actual versus model scope and the unchanged next operator action."""
import json,hashlib
from pathlib import Path
from forward_hold_plan_20261104 import ForwardHoldPlan,ROOT
from forward_hold_gate_20261104 import validate_review
from pitch_handoff_gate_20261028 import validate_loaded
O=ROOT/'right_hand_rotation_20261105/preintegration'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    request=O/'repeat/request.json';proof=O/'repeat/review.json';p=ForwardHoldPlan(request);pr=validate_review(p,proof)
    old=json.loads((ROOT/'right_hand_rotation_20261104/forward_hold/actual_review_20261105/actual_summary.json').read_bytes())
    body=json.loads((O/'body_grip/review.json').read_bytes());sweep=json.loads((O/'body_grip/whole_hand_sweep.json').read_bytes());ik=json.loads((O/'clearance_ik/review.json').read_bytes());third=json.loads((O/'third_contact/postturn_third_contact_geometry.json').read_bytes())
    pitch=validate_loaded()
    baseline=json.loads((ROOT/'right_hand_rotation_20261104/resume_review.json').read_bytes())['protected_sources_unchanged']
    if not all(sha(ROOT/f)==h for f,h in baseline.items()):raise RuntimeError('Protected source changed')
    command='& "'+str(ROOT/'leap_forward_hold_observe.cmd')+'"'
    report=dict(status='repeat_observed_forward_ready_new_layer_clear_body_grip_not_ready',
        goal='Preintegration right-hand reorientation, reliable placement, and body grip clear of upper layer',
        physical_forward_turn_and10s_hold_observations=1,physical_source=old['source'],
        source_bound_historical_pitch_observations=pitch['historical_pitch_successes'],pitch_count_is_historical_not_new_execution=True,
        current_readonly='resume_off_readonly_20261105_033008_173180/result.json',current_readonly_torque_off=True,
        current_empty_recovery=dict(static_cases=len(pr['empty_recovery']['static_cases']),dynamic_cases=len(pr['empty_recovery']['dynamic_cases']),all_passed=pr['empty_recovery']['all_passed'],review=proof.relative_to(ROOT).as_posix(),sha256=sha(proof)),
        body_grip_research=dict(lower_contact_candidates=len(body['cases']),free_cube_safety_passed=sum(bool(c.get('dynamic',{}).get('safety_passed')) for c in body['cases']),whole_upper_layer_sweep_cases=len(sweep['cases']),whole_upper_layer_sweep_clear=sum(c['all_hand_geoms_clear'] for c in sweep['cases']),whole_hand_geoms_per_case=56,clearance_endpoint_searches=len(ik['cases']),accepted_clearance_endpoints=sum(c['endpoint_proposal_accepted'] for c in ik['cases']),lower_index_contact_searches=len(third['cases']),accepted_lower_index_contacts=len(third['reachable']),model_not_registered_to_actual_cube=True,physical_load_verification=False,new_operator_motion_admitted=False),
        next_user_action_required=True,next_user_action='Repeat the identical successful forward/fixed10s trial once; no new body grip is added.',
        next_user_command=command,next_user_command_ready=True,nominal_motion_seconds=len(p.plan(p.source_position()))*.075,
        same_successful_loaded_joint_route=True,automatic_open=False,automatic_inverse=False,no_new_loaded_joint_targets=True,
        next_physical_result='A second independent approximately-quarter-turn/10s hold observation; not a lower-body hold or left-layer-load certificate.',
        future_work='Resolve middle motor clearance while retaining a lower-body contact, then validate complete free-cube path. Separate pitch/yaw connection and left-hand load tests remain.',
        assistant_motor_writes=0,protected_sources_unchanged=True)
    (O/'progress.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    note='''## CURRENT 2026-11-05: 통합 전 준비와 동일 정회전 재현 검사

- 사용자가 큐브 제거를 확인했다. `resume_off_readonly_20261105_033008_173180`에서 개별 PING16회, 일반 READ96회, Fast READ6회만 수행했다. 16개 모터 모두 OFF, 세 표본 최대 변동1틱, 상태 검사 오류 없음, 설정과 목표 쓰기0회다.
- 이 현재 빈손 자세에서 기존 정회전/10초 유지 경로로 돌아가는 형상201조건과 동역학9조건이 모두 통과했다. 명목 구동171.75초이며 기존300초 한도,450mA/PID/75ms/2틱 및 전압/통신/추종 중단 기준을 그대로 쓴다. 기존 파일로 같은 동작을 한 번 더 재현 확인할 수 있다.
- 성공한 실제 경로와 동일한 모델 큐브를 자유 상태로 재생하고 끝 상태에서 낮춰 잡는12후보를 조사했다.8후보는 기존 동역학 안전 기준을 통과했으나, 대표3후보의 윗층0~90도 회전 공간은 모두 중지 모터 및 손끝 형상과 겹쳤다. 전체56개 손 충돌 형상을 포함했다. 끝 자세만 맞추는 추가12탐색과 낮은 검지 접촉4탐색도 모든 조건을 충족하지 못했다. 모델이 실제 큐브에 정밀 등록된 것은 아니므로 실물 충돌 확정으로 해석하지 않으며 새 파지 동작은 실물 실행용으로 내보내지 않는다.
- 수직 뒤집기는 기존 원본과 영상 해시로 검증된 과거2회 기록이 있다. 별도의2026-10-28보완 뒤집기/전달1회는 초기 늦은 배치라는 제한이 있다. 이를 새 성공이나 수직/수평 연결 완료로 합산하지 않는다.
- 다음 사용자 작업은 기존 `leap_forward_hold_observe.cmd` 1회로 배치와 회전/유지의 재현 여부를 확인하는 것이다. 원래 목표 좌표는 바꾸지 않았다. 새 몸통 파지와 왼손 윗층 회전 하중 및 양손 간섭 검증은 아직 남아 있다.
- 현재 결과: `right_hand_rotation_20261105/preintegration/progress.json`. 기존 성공 영상과 원본 로그, 주요6개 보호 파일은 그대로다.

'''
    path=ROOT/'professor_progress_20261015.md';oldtext=path.read_text(encoding='utf-8')
    if not oldtext.startswith('## CURRENT 2026-11-05: 통합 전 준비와 동일 정회전 재현 검사'):
        if oldtext.startswith('## CURRENT'):oldtext=oldtext.replace('## CURRENT','## PREVIOUS',1)
        path.write_text(note+oldtext,encoding='utf-8')
    for rel in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        path=ROOT/rel;d=json.loads(path.read_bytes());d.update(status=report['status'],next_user_action_required=True,next_user_action=report['next_user_action'],next_user_command=command,next_user_command_ready=True,latest_preintegration_progress=(O/'progress.json').relative_to(ROOT).as_posix(),new_body_hold_operator_ready=False);path.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
