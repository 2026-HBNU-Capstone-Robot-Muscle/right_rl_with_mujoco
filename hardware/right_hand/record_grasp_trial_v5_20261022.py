"""Publish verified offline results and preserve the original stopped trial."""
import hashlib
import json
from pathlib import Path
import numpy as np
from grasp_place_plan import GraspPlacePlan, LOADED_PHASE, endpoints
from thumb_live_start_gate import validate_proof

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'prepared_cube_review_20261022'
FOLDER=ROOT/'cube_grasp_trial_20261022_123011_432278'


def save(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':
    tests=json.loads((BASE/'grasp_trial_v5_tests.json').read_bytes())
    if not tests['passed'] or tests['hardware_commands_sent']!=0:
        raise RuntimeError('Offline regression suite has not passed')
    proofs=[]
    for suffix in ('fixture','prior_fixture','after_stop_fixture','first_stop_fixture','loaded_stop_fixture'):
        folder=BASE/('grasp_trial_v5_'+suffix)
        plan=GraspPlacePlan(folder/'request.json');proof=validate_proof(plan,folder/'screen.json')
        integration=json.loads((folder/'windows_integration.json').read_bytes())
        if not integration['actual_windows_wsl_subprocess_passed'] or not integration['accepted']:
            raise RuntimeError('Real Windows/WSL validation missing')
        proofs.append(dict(fixture=folder.name,static_cases=len(proof['static_screens']),
            dynamic_gain_cases=[s['gain_scale'] for s in proof['dynamic_sensitivity']],
            max_dynamic_tracking_rad=max(s['max_tracking_rad'] for s in proof['dynamic_sensitivity']),
            max_unapproved_contact_mm=max(s['self_escape_max_unapproved_overlap_mm']
                for s in proof['static_screens']+proof['dynamic_sensitivity']),
            max_support_increase_mm=max(s['max_support_increase_mm'] for s in proof['static_screens']),
            stored_snapshot_only=True,current_hardware_pose_verified=False))
    result=FOLDER/'result.json';raw=result.read_bytes();r=json.loads(raw)
    samples=r['samples'];last=samples[-1]
    goals=[w for w in r['write_events'] if w['address']==116]
    opened,closed=endpoints()
    loaded=[np.rint(opened+(closed-opened)*i/322).astype(int) for i in range(1,323)]
    completed=next(i for i,q in enumerate(loaded,1) if np.array_equal(q,last['goal']))
    review=dict(source_sha256=hashlib.sha256(raw).hexdigest(),status=r['status'],error=r['error'],
        failed_phase=r['failed_phase'],all_torque_off_verified=r['all_torque_off_verified'],
        settings_restored=r['settings_restored'],contact_release_operator_confirmed=r['contact_release_operator_confirmed'],
        cube_placement_confirmed=r['cube_placement_confirmed'],cube_removal_confirmed=r.get('cube_removal_confirmed',False),
        loaded_goals_issued=completed,loaded_goals_total=322,
        min_voltage_v=min(min(s['voltage_v']) for s in samples),
        peak_current_ma=max(abs(x) for s in samples for x in s['current_ma']),
        max_tracking_rad=max(max(s['tracking_rad']) for s in samples),
        read_recovery_events=r.get('read_failures',[]),
        previous_issued_goal=goals[-2]['requested'],last_issued_goal=goals[-1]['requested'],
        actual_goal_register_snapshot_saved=False,
        diagnosis='ID1 reported the preceding command after two recent recovered read failures. Delayed response versus undelivered goal cannot be established from the original log.',
        photos_reviewed=['cube_before_approach.jpg','camera/frame_01173.jpg'],
        physical_grasp_verified=False,physical_rotation_verified=False,hardware_commands_sent_by_assistant=0)
    save(FOLDER/'stop_review.json',review)
    ready=dict(status='exact_previous_goal_fresh_confirmation_v5_ready',plan_version=GraspPlacePlan.PLAN_VERSION,
        launcher='leap_cube_grasp_trial.cmd',source_stop_review=str((FOLDER/'stop_review.json').relative_to(ROOT)),
        policy=dict(admission='entire readback equals the immediately preceding issued packet; all steps <=2 ticks',
            confirmation='two consecutive exact full-health reads of the unchanged command within 150ms/3 new reads',
            writes_during_confirmation=0,goal_write_retries=0,
            repeated_mismatch_stop='third event within 10s',
            persistent_wrong_partial_or_unrelated_goals_stop=True,
            health_held_joint_and_progress_checks_on_every_snapshot=True,
            initial_enable_checks_stay_strict=True),
        loaded_goals_unchanged=True,current_pid_voltage_tracking_stops_unchanged=True,
        tests=tests,windows_wsl_saved_cases=proofs,hardware_commands_sent_by_assistant=0,
        physical_grasp_verified=False,physical_rotation_verified=False,live_sac=False,
        user_task='큐브를 빼고 같은 명령 실행. 빈손 첫 SPACE → 약지 끝/손바닥 분리 확인 후 두 번째 SPACE → PLACE에 큐브 배치/손 제거/세 번째 SPACE → REMOVE에서 큐브 제거/네 번째 SPACE. 분리가 안 되거나 불확실하면 Q. 로그 전달.')
    save(BASE/'grasp_trial_v5_ready.json',ready)
    section=f'''## CURRENT 2026-10-22 12:30 큐브 접근 중 목표 읽기 불일치 / 새 읽기 확인 v5

- v4 실제 실행은 약지 분리 사용자 확인·빈손 열기·큐브 배치까지 통과. loaded 접근322개 중{completed}개 목표를 보낸 뒤 `ID1 goal readback 2784 != command 2786`에서 중단. 최저{review['min_voltage_v']:.1f}V, 최대{review['peak_current_ma']}mA, 최대 추종{review['max_tracking_rad']:.5f}rad. 전체 토크 OFF/설정 복원 확인. 실물 파지 완주·회전은 아직 미검증.
- 직전 읽기 두 번에서 corrupt/timeout 후 새 읽기 복구가 기록됐고,2784는 바로 앞 명령이다. 지연 응답인지 목표 전송 누락인지는 기존 기록만으로 확정하지 않음. 기존 샘플은 목표 레지스터 전체를 저장하지 않아 원인을 단정할 수 없으며, 원본 결과는 보존하고 `stop_review.json`에 근거를 정리.
- v5는 전체 읽기 목표가 정확히 직전 전송 벡터이고 이동폭2틱 이내인 경우에만 목표를 고정한 채 추가 읽기.150ms/최대3개 새 전체 상태 안에 현재 명령과 정확히 일치하는 값이 연속2회 나와야 같은 다음 목표로 진행. 목표 재전송·기준값 변경·2틱 오차 허용은 없음. 계속 불일치/일부만 불일치/예상 밖 목표/다른 건강 상태 실패/10초 내3번째 불일치는 다음 목표 없이 OFF/복원.
- 명령값과 목표 레지스터 읽기값을 이제 따로 기록. 추가 읽기 중에도 매번 전류·전압·토크·게인·추종·고정 관절·실측 진행 검사. 초기 preload/enable 검사,450mA/PID/0.18rad/55초 제한 및322개 loaded 목표 유지.
- {tests['tests_run']}개 오프라인 테스트 통과. 일시 이전 응답은 원래 목표 큐를 그대로 완료하며, 지속 불일치는 목표 횟수가 늘지 않고 OFF/복원됨을 검증. v5 저장 시작5개(이번 loaded 중단 뒤 OFF 포함) 각각67개 정적 경로·세 강도 동역학 Windows→WSL 실제 검사 통과. 저장 OFF 표본은 현재 안정 자세의 인증이 아니며 실행 때 다시 새로 읽어 검사. 조수 모터/카메라 접근 없음.
- 다음 사용자 작업: 큐브를 빼고 같은 `leap_cube_grasp_trial.cmd` v5. 빈손 첫 SPACE → 약지 끝이 손바닥과 떨어진 경우만 두 번째 SPACE → PLACE에 초록 위/주황 카메라 쪽 배치·손 제거·세 번째 SPACE → REMOVE에서 큐브 제거·네 번째 SPACE. 닿거나 분리가 안 보이면Q. 결과 로그 전달.
- 근거: `cube_grasp_trial_20261022_123011_432278/stop_review.json`, `prepared_cube_review_20261022/grasp_trial_v5_ready.json`, `grasp_trial_v5_tests.json`.

'''
    for filename in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/filename;text=path.read_text(encoding='utf-8-sig')
        if not text.startswith('## CURRENT 2026-10-22 12:30'):
            path.write_text(section+text,encoding='utf-8')
    for relative in ('prepared_cube_review_20261022/current_status.json',
                     'right_hand_start_20261019/objective_and_schedule.json'):
        path=ROOT/relative;doc=json.loads(path.read_text(encoding='utf-8-sig'))
        doc.update(status=ready['status'],latest_update='2026-10-22 v5',
            latest_hardware_stop_review=ready['source_stop_review'],
            latest_ready_record='prepared_cube_review_20261022/grasp_trial_v5_ready.json',
            next_user_task=ready['user_task'],physical_grasp_verified=False,physical_rotation_verified=False)
        if relative.endswith('current_status.json'):
            doc['next_trial_evidence']='prepared_cube_review_20261022/grasp_trial_v5_ready.json'
            doc['latest_user_readback_stop']=review
        else:
            doc.update(current_user_task=ready['user_task'],
                current_blocker='v4 passed release confirmation and empty opening, then stopped during the loaded approach on a previous-goal readback. v5 confirms exact fresh reads without goal retransmission. Full physical grasp and rotation remain unverified.',
                latest_user_hardware_trial=dict(source=str(result.relative_to(ROOT)),status=r['status'],
                    error=r['error'],loaded_goals_issued=completed,loaded_goals_total=322,
                    all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored']),
                latest_offline_review='prepared_cube_review_20261022/grasp_trial_v5_loaded_stop_fixture/windows_integration.json',
                next_trial_review='prepared_cube_review_20261022/grasp_trial_v5_ready.json')
        save(path,doc)
    print(json.dumps(dict(status=ready['status'],tests=tests['tests_run'],
        saved_start_cases=len(proofs),loaded_goals_issued=completed),ensure_ascii=False))
