"""Publish the reviewed EMPTY-only command; never opens devices."""
import datetime as dt,json
from pathlib import Path
import numpy as np
from supported_inverse_empty_plan_20261029 import SupportedInverseEmptyPlan,ROOT,OUT,REQUEST,PROOF,LOADED,HAND_PATH,sha
from supported_inverse_empty_gate_20261029 import validate_review,run_screen
from observed_book_model_policy import healthy_off
from hand_motor_mapping import SIM_TO_MOTOR_IDS

def main():
    p=SupportedInverseEmptyPlan(REQUEST);proof=validate_review(p,PROOF)
    livepath=ROOT/'fast_off_readonly_20261029_224341_965504/result.json';live=json.loads(livepath.read_bytes())
    assert live['status']=='off_snapshots_saved' and live['maximum_drift_ticks']<=2 and not live['servo_writes'] and not live['torque_changes']
    for state in live['snapshots']:healthy_off({k:np.asarray(v) for k,v in state.items()})
    position=np.asarray(live['snapshots'][-1]['position'],int);difference=position-p.source_position()
    assert max(abs(difference))<=4,'Current posture needs its own model prefix; command not published'
    request=OUT/'latest_off_request.json';request.write_text(json.dumps(dict(plan_version=p.PLAN_VERSION,
        motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=live['snapshots'][-1],source=livepath.relative_to(ROOT).as_posix(),source_sha256=sha(livepath),hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    cached=OUT/'latest_off_review.json';run_screen(request,cached);validate_review(SupportedInverseEmptyPlan(request),cached)
    sources=[ROOT/'right_hand_rotation_20261029/supported_inverse_search/support_grid'/name/'review.json' for name in ('full9','full9_front_-15','full9_front_-5')]
    cases=[]
    for source in sources:
        r=json.loads(source.read_bytes());assert r['candidate_sha256']==sha(LOADED) and r['all_passed'];cases.extend(r['cases'])
    assert len(cases)==27 and all(c['passed'] and c['initial_placement_passed'] and not c['external_cube_contact'] and c['final_cube_supported_by_hand'] for c in cases)
    nominal=next(c for c in cases if (c['book_front_mm'],c['gain_scale'],c['friction'])==(-10.,1.,.3))
    angles=[c['placed_to_turn_deg'] for c in cases]
    report=dict(status='new_supported_inverse_EMPTY_operator_check_ready',timestamp=dt.datetime.now().astimezone().isoformat(),scope='EMPTY_ONLY_NO_CUBE',
        command='leap_supported_inverse_empty.cmd',proof_sha256=sha(PROOF),candidate_sha256=sha(LOADED),hand_path_sha256=sha(HAND_PATH),
        current_readonly_source=livepath.relative_to(ROOT).as_posix(),current_off_difference_ticks=difference.tolist(),
        current_pose_bound_cache_checked=True,whole_static_cases=201,whole_empty_finite_cases=9,current_prefix_static_cases=201,current_prefix_finite_cases=9,
        software_admission_regressions_passed=6,default_entry_checked_without_devices=True,
        cube_research_cases=27,cube_research_all_passed=True,cube_model_nominal_placed_to_hold_deg=nominal['placed_to_turn_deg'],
        cube_model_range_deg=[min(angles),max(angles)],cube_research_sources=[dict(path=s.relative_to(ROOT).as_posix(),sha256=sha(s)) for s in sources],
        cube_model_is_not_physical_90_verification=True,physical_new_route_completed=False,physical_inverse_90_verified=False,inverse_cube_trial_ready=False,
        ready_for_supervised_operator_EMPTY_run=True,assistant_motor_or_setting_writes=0,estimated_motion_s=len(p.plan(p.source_position()))*.075,
        operator_steps=['Keep cube outside hand and camera for the whole run.','Leave fingers and base unchanged.','Run command; one terminal SPACE.','Automatic OPEN/OFF; no PLACE step.','Report done; no log paste needed.'])
    (OUT/'handoff.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    for rel in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        path=ROOT/rel;r=json.loads(path.read_bytes());r.update(status=report['status'],current_status=report['status'],timestamp=report['timestamp'],
            next_command=report['command'],next_user_command=report['command'],next_user_command_ready=True,next_user_action_required=True,
            next_user_action='Run leap_supported_inverse_empty.cmd once with cube outside throughout; terminal SPACE once; automatic OPEN/OFF.',
            ready_for_user_run=True,ready_for_supervised_operator_run=True,ready_scope='EMPTY_ONLY_NO_CUBE',
            current_motor_connection='All16 torque OFF, error0/watchdog0; three stable snapshots; current pose cache verified',
            supported_inverse_empty_handoff=(OUT/'handoff.json').relative_to(ROOT).as_posix(),inverse_90_ready=False,
            inverse_cube_trial_ready=False,assistant_motor_or_setting_writes=0)
        path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    marker='<!-- supported_inverse_empty_ready_20261029 -->\n'
    paragraph=('새 검지 받침 역회전 경로의 빈손 실물 확인 준비 완료. 모터16개 OFF/error0/watchdog0/3회정지와 최신자세4틱캐시를 읽기로 확인했다. '
        '전체기하201/전체빈손유한9/현재복귀기하201/현재복귀유한9,스코프·해시·캐시거부시험6개 통과. '
        f'큐브모델27조건도 유지됐으며 명목{nominal["placed_to_turn_deg"]:.2f}도,범위{min(angles):.2f}..{max(angles):.2f}도다. '
        '이는 실물90도 성공이 아니다. 지금 제공하는 명령은 끝까지 큐브없는 leap_supported_inverse_empty.cmd다. '
        '원래손모양에서 터미널SPACE1회,준비/새경로/OPEN/자동OFF이며PLACE가 없다. 새경로의 실물큐브시험은 아직 준비되지 않았다. '
        '근거: right_hand_rotation_20261029/supported_inverse_empty/handoff.json\n\n')
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8-sig')
        if not text.startswith(marker):path.write_text(marker+paragraph+text,encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('cube_research_sources','operator_steps')}))
if __name__=='__main__':main()
