"""Record the review outcome and actual missing connection, preserving evidence."""
import datetime as dt,hashlib,json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261028/two_axis_connection'
def read(path):return json.loads(path.read_bytes())
def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    comparison=read(OUT/'loaded_join/route_comparison.json')
    assert len(comparison['cases'])==25 and comparison['promising']==0
    assert all(r['stop'] is None for r in comparison['cases'])
    assert all(r['max_self_mm']==0 and r['external_cube_contact'] is False for r in comparison['cases'])
    search=OUT/'seating_control_search';z=np.load(search/'candidate.npz')
    nonzero=int(np.count_nonzero(np.any(z['goals']!=z['start'],axis=1)))
    assert nonzero==0
    write(search/'outcome_review.json',dict(status='fixed_hold_only_not_an_optimized_seating_solution',
        nonzero_control_goals=nonzero,fixed_hold_goals=len(z['goals']),
        model_progress_can_be_passive_settling=True,physical_execution_allowed=False))
    protected={
        'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
        'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
        'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
        'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d',
    }
    for name,expected in protected.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected,name
    state=read(OUT/'current_progress.json')
    state.update(timestamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat(),
        status='connection_model_review_saved_waiting_for_robot_usb',
        loaded_transfer_order_cases=25,loaded_transfer_order_candidates_accepted=0,
        best_nominal_transfer_seat_error_mm=min(r['seat_error_to_yaw_initial_mm'] for r in comparison['cases']),
        existing_grasp_open_seating_cycle_rejected_in_model=True,
        control_optimization_yielded_only_fixed_holds=True,
        image_orientation_diagnostic_is_uncertain_not_accepted_registration=True,
        idealized_orientation_routes=24,logical_maximum_quarter_turn_steps=5,
        logical_routes_are_not_a_physical_connection_certificate=True,
        latest_readonly='prefix638_off_readonly_20261028_205710_167004/result.json',
        robot_usb_adapter_currently_missing=True,
        next_user_action='Remove cube; reconnect robot USB and existing5V motor power and switch on. Leave fingers/support unchanged. Do not run a motion command.',
        next_user_command=None,next_after_connection='Read fresh OFF angles only, then screen the next empty/pitch-transfer path.',
        new_operator_command_ready=False,protected_sources_unchanged=True)
    write(OUT/'current_progress.json',state)
    for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
                 'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        write(ROOT/name,state)
    text='''2026-10-28 20:57 현재 연결 확인: 알려진 로봇 USB 어댑터가 목록에 없어 상태 읽기를 시작하지 못했다. 기록은 `prefix638_off_readonly_20261028_205710_167004/result.json`이며 목표/토크/설정 쓰기0회다. 사용자에게 큐브를 빼둔 채 USB와 기존5V 전원을 연결하고 켜달라고 요청했으며, 동작 프로그램 실행은 요청하지 않았다. 연결 뒤 새 OFF 관절값을 읽어 다음 경로의 실제 출발점을 정한다.

연결 경로 물리 비교25가지는 모두 큐브 유지와 손 자체의 충돌 조건은 지켰으나 다음 파지 기준 위치까지 이동하지 못했다. 가장 가까운 경우도 약17.9mm가 남았다. 단순 연결 및 잡기/펴기1회 뒤 기존 회전까지 붙인 경로는 모델에서 큐브 이탈이 있어 실물 실행용으로 채택하지 않았다. 제어 최적화의720개 결과는 모두 같은 고정 목표였으므로 새 재파지 동작이 아니라 수동 정착/대기의 효과로 별도 표시했다. 실물 사진의 출발 각도 비교도 적합 오차 때문에 아직 좌표 등록으로 받아들이지 않았다. 실패 조건을 완화하거나 기존 성공 코드/관절 한계를 변경하지 않았다.

두 관찰 방향을 이상적인90도 동작으로 취급한 논리 계획에서는 모든24개 큐브 방향에 도달하고 최장5회가 필요함을 확인했다. 이는 실물 연결 성공이나 정확한90도 제어 인증이 아니다. 실제 완료 근거는 과거 수직 목표 면 전환2회와 오늘 수평 회전 고정 약88도/OPEN 뒤 약82도라는 기존 관찰이다. 새 실물 동작 명령은 아직 준비하지 않았다.

<!-- END TWO AXIS USB HANDOFF 20261028 -->

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig')
        if '<!-- END TWO AXIS USB HANDOFF 20261028 -->' not in old:path.write_text(text+old,encoding='utf-8')
    print(json.dumps({k:state[k] for k in ('status','loaded_transfer_order_cases','best_nominal_transfer_seat_error_mm','protected_sources_unchanged','new_operator_command_ready')},ensure_ascii=False))

if __name__=='__main__':main()
