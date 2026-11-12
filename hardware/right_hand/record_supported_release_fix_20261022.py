"""Record evidence for the revised EMPTY release; no hardware access."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261022'

def main():
    from supported_yaw_empty_plan import SupportedYawEmptyPlan
    from thumb_live_start_gate import validate_proof
    native=json.loads((OUT/'integration_validation.json').read_bytes())
    tests=json.loads((OUT/'camera_signal_fix_tests.json').read_bytes())
    assert native['passed'] and len(native['fixtures'])==18
    assert tests['passed']
    folder=OUT/'fixture_supported_failed_sag'
    proof=validate_proof(SupportedYawEmptyPlan(folder/'request.json'),folder/'screen.json')
    failed=ROOT/'supported_yaw_empty_20261022_155659_749370'
    original=json.loads((failed/'result.json').read_bytes())
    oldproof=json.loads((failed/'current_start_screen.json').read_bytes())
    assert original['write_events']==[] and original['all_torque_off_verified']
    assert not oldproof['accepted'] and proof['accepted']
    report=dict(status='revised_EMPTY_rehearsal_ready_for_operator',
        source_run=str(failed.relative_to(ROOT)),
        source_result_sha256=hashlib.sha256((failed/'result.json').read_bytes()).hexdigest(),
        source_hardware_write_count=0,source_torque_off_verified=True,
        source_camera_good_frames=original['camera_recording']['image_signal_usable_frames'],
        source_camera_blank_frames=original['camera_recording']['image_signal_blank_frames'],
        actual_failure='Low-gain dynamics recontacted the same ring-tip/palm pair at 2.51s, after the 2.5s separation deadline.',
        modeled_recontact_mm=oldproof['dynamic_sensitivity'][0]['self_escape_max_unapproved_overlap_mm'],
        diagnostic_bug='Derived EMPTY plan used the historical 64-tick diagnostic limit; its active certificate limit was 80.',
        plan_version=SupportedYawEmptyPlan.PLAN_VERSION,
        revision='For PIP recovery travel beyond 80 ticks, add up to 64 ticks of ring-tip extension in the initial EMPTY release. Short releases and the subsequent suffix remain unchanged.',
        fixture_static_cases=len(proof['static_screens']),fixture_dynamic_cases=len(proof['dynamic_sensitivity']),
        maximum_static_separation_ticks=max(r['contact_clear_pip_travel_ticks'] for r in proof['static_screens']),
        dynamics=proof['dynamic_sensitivity'],
        native_ranges_unchanged=True,release_deadline_s=2.5,release_travel_limit_ticks=80,
        current_cap_ma=450,maximum_goal_step_ticks=2,motion_deadline_s=75,
        windows_wsl_fixtures_passed=len(native['fixtures']),tests_passed=tests['tests_run'],
        agent_hardware_commands_sent=0,cube_allowed=False,loaded_path_approved=False,
        physical_rotation_verified=False,live_sac=False,
        next_action='Cube OUT. Leave the hand as-is. Run leap_supported_yaw_empty.cmd; SPACE, observe ring-tip/palm separation and SPACE. Automatic OFF after rehearsal; Q stops.')
    (OUT/'supported_release_fix_review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    ready_path=OUT/'supported_empty_ready_review.json'
    ready=json.loads(ready_path.read_bytes())
    ready.update(status=report['status'],plan_version=report['plan_version'],
        superseded_by_review='right_hand_rotation_20261022/supported_release_fix_review.json',
        native_start_fixtures=4,unique_tests_covered=tests['tests_run'],
        existing_regression_tests_passed=tests['tests_run']-9,
        new_empty_tests_passed=9,next_user_action=report['next_action'])
    ready_path.write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
    section=f'''## CURRENT 2026-10-22 15:56 이후: 빈손 v2 분리 경로 수정, 실물 재검사 필요

- 마지막 두 `supported_yaw_empty` 실행은 OFF 시뮬레이션 단계에서 중단됐고 하드웨어 쓰기0개, 토크OFF 확인. 마지막 촬영405프레임은 모두 정상이다. 앞선15:53 두 시도에서는 일시적인 검은 영상/장치 누락도 있었으나 마지막 실패 원인은 카메라가 아니다.
- 원본 정적67개는 모두 통과했고 분리 최대80틱도 활성 기준 안이었다. 약한 응답0.5배 동역학에서2.51초에 기존 약지 끝/손바닥 쌍이 약0.0011mm 재접촉하여2.5초 분리 기준에 걸렸다. 파생 실행본의 실패 메시지가 과거64틱 기준을 쓰던 문제도 수정했다. 실제 접촉압이나 실물 충돌 깊이로 해석하지 않는다.
- 빈손 v2에서 ID15(PIP)의 원래 복귀 거리가80틱을 넘으면 ID13(약지 끝)도최대64틱(약5.6도) 펴서 빠져나오도록 변경. 짧은 복귀에서는 이전 목표와 고정관절 검사를 유지한다. 엄지/다른 관절의 순서, 이후256개 새 경로, 전류450mA, 목표당2틱/50ms,75초·분리2.5초/80틱, 추종/진행/전압/카메라 중단 및OFF 복원 기준은 유지. 기존 적재v6의 목표 경로는 동일하다.
- 실패한 실제 저장 자세를 포함한 새 빈손4개 시작은 각각67개 정적+3개 응답 동역학 검사를 통과했다. 공유 코드 영향까지18개 시작을 Windows→WSL에서 재검사했고, 장치 대체 회귀{tests['tests_run']}개가 통과했다. 조수가 실제 모터/카메라를 열지 않았다.
- 사용자 할 일: 큐브 없이 현재 손을 그대로 두고 `leap_supported_yaw_empty.cmd` 실행. 첫SPACE로 시작, 약지 끝이 손바닥에서 떨어진 것을 눈으로 확인한 뒤SPACE. 새 동작 뒤 자동OFF한다. 닿거나 걸리면Q. 긴 로그를 붙이지 않아도 했어만 알리면 저장 결과를 읽는다. 적재 회전/90도 실물 성공이나 SAC 성공은 아직 없다.
- 근거: `right_hand_rotation_20261022/supported_release_fix_review.json`, `integration_validation.json`, `camera_signal_fix_tests.json`. 원본 실패 결과는 보존했다.

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        path.write_text(section+old,encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('status','plan_version','windows_wsl_fixtures_passed','tests_passed','cube_allowed')},indent=2))

if __name__=='__main__':main()
