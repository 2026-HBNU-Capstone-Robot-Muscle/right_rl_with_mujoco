"""Record only completed offline checks; never opens real devices."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261022'


def load(path):return json.loads(path.read_bytes())


def main():
    candidate=OUT/'small_rotation_trial_candidate.json'
    replay=load(OUT/'small_yaw_refined_result_independent.json')
    native=load(OUT/'rotation_integration_validation.json')
    tests=load(OUT/'trial_tests.json')
    regression=load(ROOT/'prepared_cube_review_20261022/grasp_trial_v6_tests.json')
    assert replay['clearance_tracking_screen_passed'] and replay['total_cases']==77
    assert native['passed'] and len(native['fixtures'])==3
    assert tests['passed'] and tests['tests_run']==9
    assert regression['passed'] and regression['tests_run']==126
    motion=[r for r in replay['trials'] if not r['control']]
    nominal=[r for r in motion if r['gain']==1 and r['cube_friction_scale']==1 and r['mass_kg']==.1]
    result=dict(status='operator_small_rotation_observation_ready_NOT_physical_success',
        source_trial='cube_grasp_trial_20261022_141534_666300',
        user_confirmed_unassisted_retention_during_approach=True,
        previous_trial_approach_only=True,physical_rotation_verified=False,
        algorithm='offline_CEM_fixed_path_NOT_SAC',live_sac=False,
        candidate_file=str(candidate.relative_to(ROOT)),candidate_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),
        rotation_goals=80,rotation_nominal_seconds=4,maximum_additional_joint_travel_ticks=96,
        offline_clearance_tracking_cases_passed=77,offline_requested_10deg_cases_passed=replay['successful_cases'],
        requested_10deg_cases_total=77,requested_rotation_passed=False,
        nominal_cube_yaw_range_deg=[min(r['projected_yaw_deg'] for r in nominal),max(r['projected_yaw_deg'] for r in nominal)],
        all_case_cube_yaw_range_deg=[min(r['projected_yaw_deg'] for r in motion),max(r['projected_yaw_deg'] for r in motion)],
        native_start_fixtures=3,static_cases_per_start=67,dynamic_gains_per_start=3,
        fake_device_tests_passed=135,
        current_off_delay_replay_within_55_seconds=True,
        extra_pre_rotation_wait_removed=True,hard_motion_deadline_seconds=55,
        real_cube_registration_verified=False,hardware_commands_sent_by_agent=0,
        next_user_action='Run leap_cube_small_rotation_trial.cmd with empty hand; PLACE only at prompt. Watch front face/edges during small yaw, remove at prompt. Reply 했어.',
        launcher='leap_cube_small_rotation_trial.cmd',
        simulation_gif='right_hand_rotation_20261022/small_yaw_refined_result_replay.gif',
        limitations=['Cube pose resets are provisional; exact physical rotation is uncertain.',
                    'Native actuators approximate real current/PID/contact behavior.',
                    'The first observation is a small yaw, not a 90-degree flip.'])
    (OUT/'trial_ready_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    section='''## CURRENT 2026-10-22: 첫 소회전 관찰 실행본 준비 / 실제 회전은 미검증

- 사용자는 14:15 v6 접근 중 큐브를 사람 손으로 받치거나 바로잡을 일이 없었다고 확인했다. 당시 프로그램은 쥐기 접근만 했고 회전 명령은 없었다.
- 새 `leap_cube_small_rotation_trial.cmd`는 같은 빈손 복귀·약지 분리 확인·OPEN·PLACE·완료된 파지 접근 뒤, 고정된 작은 수평 회전 경로80개(명목4초)를 실행하고 REMOVE/OFF한다. 90도 회전이나 실시간 SAC가 아니다. 추가 관절 이동은96틱 이하, 목표당2틱·450mA/PID·전압/전류/진행/추종·55초·OFF 복원 조건을 유지했다.
- CEM 탐색 결과의 작은 후보를 별도 투영 각도 계산과 무동작 대조군으로 재검사. 큐브 위치7개·gain/마찰·80/120g 민감도 등77개에서 충돌/받침/낙하/추종 기준 통과. 요청한10도 회전 기준은14/77개만 통과했고, 실제 큐브 등록·파지력·회전 성공을 인증하지 않는다. 이 불확실성을 숨기거나 10도 달성 기준을 낮추지 않았다. 다음 단계는 제한된 실물 회전량 관찰이다.
- 저장된 시작 자세3개에 각각67개 정적 및3개 강도 동역학 경로 검사 통과. 기존 경로11개도 실제 Windows→WSL 재검사. 기존126개와 신규/보고9개, 총135개 가짜 장치 검사 통과. 처리 지연을 넣은 현재 OFF 자세 재생에서55초 이내 완료. 불필요하게 추가했던 회전 전1초 대기만 제거했고 이동 속도·hard deadline은 바꾸지 않았다.
- 조수는 모터·카메라를 실행하지 않았다. 사용자가 새 cmd를 실행할 차례다. 큐브는PLACE 때만 놓고, 접근 뒤 수평 회전 중 앞면·모서리를 본다. 윗면 색이 유지되어도 수평 회전은 가능하다. REMOVE 때 빼고SPACE. 긴 로그 대신 했어와 실제 움직임만 알리면 저장 결과/사진을 읽는다.
- 근거: `right_hand_rotation_20261022/trial_ready_review.json`, `rotation_integration_validation.json`, `trial_tests.json`, `small_yaw_refined_result_independent.json`. 시뮬레이션은 `small_yaw_refined_result_replay.gif`.

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8')
        if not text.startswith(section):path.write_text(section+text,encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('status','nominal_cube_yaw_range_deg','fake_device_tests_passed','launcher')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
