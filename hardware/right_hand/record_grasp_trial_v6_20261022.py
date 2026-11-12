"""Record the reproduced v5 screen stops and verified v6 software correction."""
import hashlib
import json
from pathlib import Path
from grasp_place_plan import GraspPlacePlan
from thumb_live_start_gate import validate_proof
from review_ring_release_screen_20261022 import ROOT,FOLDERS

BASE=ROOT/'prepared_cube_review_20261022'


def save(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':
    tests=json.loads((BASE/'grasp_trial_v6_tests.json').read_bytes())
    if not tests['passed']:raise RuntimeError('Regression suite has not passed')
    cases=[]
    for suffix in ('fixture','prior_fixture','after_stop_fixture','first_stop_fixture',
                   'loaded_stop_fixture','125440_fixture','125613_fixture'):
        folder=BASE/('grasp_trial_v6_'+suffix);request=json.loads((folder/'request.json').read_bytes())
        plan=GraspPlacePlan(folder/'request.json');p=validate_proof(plan,folder/'screen.json')
        original=ROOT/request['offline_replay_source'];old=original.parent/'screen.json'
        if not old.exists():old=original.parent/'current_start_screen.json'
        old=json.loads(old.read_bytes())
        if old['plan_sha256']!=p['plan_sha256']:
            raise RuntimeError('A motion sequence changed')
        integration=json.loads((folder/'windows_integration.json').read_bytes())
        if not integration['actual_windows_wsl_subprocess_passed']:raise RuntimeError('Real WSL check missing')
        cases.append(dict(fixture=folder.name,accepted=p['accepted'],static_cases=67,dynamic_cases=3,
            motion_goals_identical_to_v5=True,
            max_release_pip_travel_ticks=max(s['contact_clear_pip_travel_ticks'] for s in p['static_screens']),
            last_static_separation_goal=max(s['self_escape_complete_at'] for s in p['static_screens']),
            last_dynamic_separation_seconds=max(s['self_escape_complete_at'] for s in p['dynamic_sensitivity']),
            max_unapproved_self_contact_mm=max(s['self_escape_max_unapproved_overlap_mm']
                for s in p['static_screens']+p['dynamic_sensitivity']),
            native_ranges_unchanged=p['native_ranges_unchanged'],stored_snapshot_only=True))
    reviews=[]
    for name in FOLDERS:
        folder=ROOT/name;data=(folder/'result.json').read_bytes();r=json.loads(data)
        proof=json.loads((folder/'current_start_screen.json').read_bytes())
        if r['write_events'] or not r['all_torque_off_verified']:raise RuntimeError('Unexpected hardware outcome')
        review=dict(source_sha256=hashlib.sha256(data).hexdigest(),source=str((folder/'result.json').relative_to(ROOT)),
            original_status=r['status'],original_error=r['error'],write_attempts=0,
            all_torque_off_verified=True,settings_restored=r['settings_restored'],
            restoration_needed=False,off_position=proof['start_ticks'],
            nominal_release_pip_travel_ticks=proof['static_screens'][0]['contact_clear_pip_travel_ticks'],
            maximum_release_pip_travel_ticks=max(s['contact_clear_pip_travel_ticks'] for s in proof['static_screens']),
            maximum_static_separation_goal=max(s['self_escape_complete_at'] for s in proof['static_screens']),
            maximum_dynamic_separation_seconds=max(s['self_escape_complete_at'] for s in proof['dynamic_sensitivity']),
            cause='Only the additional 64-tick first-separation cutoff failed. Monotone separation, no new contacts, native ranges, support clearance, <=40 goals and <=2.5s native separation all passed.',
            original_results_preserved=True,physical_release_in_this_pose_verified=False)
        save(folder/'stop_review.json',review);reviews.append(review)
    task='큐브를 빼고 같은 leap_cube_grasp_trial.cmd v6 실행. 빈손 첫 SPACE → 약지 끝이 손바닥에서 떨어진 것을 확인한 경우만 두 번째 SPACE → PLACE에서 초록 위/주황 카메라 쪽 배치·사람 손 제거·세 번째 SPACE → REMOVE에서 큐브 제거·네 번째 SPACE. 분리가 안 되거나 안 보이면 Q. 로그 전달.'
    ready=dict(status='release_budget_consistency_v6_ready',plan_version=GraspPlacePlan.PLAN_VERSION,
        launcher='leap_cube_grasp_trial.cmd',source_stops=reviews,
        previous_extra_first_separation_limit_ticks=64,
        first_separation_goal_window=40,maximum_goal_step_ticks=2,
        derived_first_separation_limit_ticks=GraspPlacePlan.MAX_CONTACT_RELEASE_TRAVEL_TICKS,
        first_separation_time_limit_seconds=2.5,
        measured_release_margin_ticks=16,operator_release_confirmation_required=True,
        command_sequences_unchanged=True,current_pid_voltage_tracking_stops_unchanged=True,
        exact_previous_goal_fresh_confirmation_preserved=True,
        physical_grasp_verified=False,physical_rotation_verified=False,live_sac=False,
        loaded_cube_dynamics_revalidated_this_turn=False,hardware_commands_sent_by_assistant=0,
        tests=tests,windows_wsl_saved_cases=cases,next_user_task=task)
    save(BASE/'grasp_trial_v6_ready.json',ready)
    section=f'''## CURRENT 2026-10-22 12:54/12:56 분리 travel 검사 중단 / 동일 경로 v6

- 두 v5 실행 모두 모터 설정/목표 쓰기0회,16개 토크 OFF 확인. `settings_restored=False`는 복원할 쓰기가 없기 때문이며 복원 실패가 아니다. 원본 stopped 결과·failed screen은 보존하고 각 폴더에 `stop_review.json` 추가.
- 마지막 ON 직후의 OFF 표본과 달리 약지 PIP가2851틱까지 접힌 현재 자세. 두 실행의 시작 각도 차이는 ID12의1틱뿐이고 나머지15개 동일. 이 두 표본만으로 처짐의 물리 원인을 단정하지 않음. 현재 모델의 최초 약지 끝/손바닥 겹침 nominal2.728mm, ±4틱 사례 최대3.054mm는 실물 눌림 깊이를 측정한 값이 아니다.
- 저장된 두 failed screen은 기존 접촉의 단조 감소·새/더 깊은 접촉 금지·관절 범위·받침 조건을 모두 통과. 최초 분리는 nominal66틱/33개 목표, 가장 늦은 사례74틱/37개 목표, 동역학 최대2.21초로 기존40개/2.5초 창 안이다. 별도로 설정했던64틱 보수 조건만47/67개 사례에서 실패하여 모터 쓰기 전 거부됐다.
- v6는 최초 분리 travel 상한을 기존40개 목표 × 목표당2틱 =80틱으로 도출. 임의 시작에서 무제한 동작을 허용하지 않음. 증명서의 travel이 실제 분리 목표 수×2틱 이내인지도 추가 검사해 서로 모순된 수치를 거부. 진단 메시지도 같은 active plan 한도를 참조. 이동 목표·순서·속도·450mA/PID·전압/추종/실측 진행/55초/OFF 복원·v5의 목표 읽기 재확인 동작은 변경하지 않음. 실제 약지 분리 각도 읽기와 사용자의 분리 확인 SPACE도 계속 필요.
- 이전 자세5개와 이번 두 시작 자세, 총7개 Windows→WSL 검사 통과(각67개 정적 시작 및세 강도 동역학). 모든 명령 경로 해시가 각 v5 출처와 동일. 현재 자세의 새로운/허용되지 않은 자기 접촉·받침/범위 위반0. 이 검사는 빈손 복귀 경로이며 실제 큐브 파지/회전 인증이 아니다. {tests['tests_run']}개 테스트 통과.
- 다음 사용자 작업: 큐브를 빼고 같은 명령 v6 실행. 빈손 첫 SPACE → 약지 실제 분리 확인 후 두 번째 SPACE → PLACE 후 배치/손 제거/세 번째 SPACE → REMOVE 후 제거/네 번째 SPACE. 분리가 불확실하면 Q. 새 사진·손가락 수동 재배치 불필요.
- 근거: 두 실패 폴더 `stop_review.json`, `prepared_cube_review_20261022/grasp_trial_v6_ready.json`, `grasp_trial_v6_tests.json`.

'''
    for filename in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/filename;text=path.read_text(encoding='utf-8-sig')
        if not text.startswith('## CURRENT 2026-10-22 12:54/12:56'):
            path.write_text(section+text,encoding='utf-8')
    for relative in ('prepared_cube_review_20261022/current_status.json',
                     'right_hand_start_20261019/objective_and_schedule.json'):
        path=ROOT/relative;doc=json.loads(path.read_text(encoding='utf-8-sig'))
        doc.update(status=ready['status'],latest_update='2026-10-22 v6',next_user_task=task,
            latest_ready_record='prepared_cube_review_20261022/grasp_trial_v6_ready.json',
            latest_user_screen_stops=reviews,physical_grasp_verified=False,physical_rotation_verified=False)
        if relative.endswith('current_status.json'):
            doc.update(next_trial_evidence='prepared_cube_review_20261022/grasp_trial_v6_ready.json',
                       latest_user_screen_stop=reviews[-1])
        else:
            doc.update(current_user_task=task,
                current_blocker='The two v5 starts failed only an extra 64-tick separation cutoff before writes. The same paths pass the v6 derived 40-goal/80-tick budget. Awaiting physical loaded approach completion; no rotation verified.',
                latest_user_hardware_trial=dict(source=reviews[-1]['source'],status='stopped_before_writes',
                    write_attempts=0,all_torque_off_verified=True),
                latest_offline_review='prepared_cube_review_20261022/grasp_trial_v6_125613_fixture/windows_integration.json',
                next_trial_review='prepared_cube_review_20261022/grasp_trial_v6_ready.json')
        save(path,doc)
    print(json.dumps(dict(status=ready['status'],tests=tests['tests_run'],saved_pose_cases=len(cases),
        maximum_current_release_ticks=74,allowed_release_ticks=80,motion_sequences_changed=False)))
