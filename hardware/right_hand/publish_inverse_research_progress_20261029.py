"""Record observed motion and research failures without granting motor access."""
import datetime as dt,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/contact_retained_inverse'
ACTUAL=ROOT/'gapped_inverse_trial_20261029_184252_272166/offline_review/physical_inverse_review.json'
PING=ROOT/'connection_ping16_readonly_20261029_220204_262459/result.json'
def load(path):return json.loads(path.read_bytes())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
    actual=load(ACTUAL);ping=load(PING);geom=load(OUT/'whole_geometry/review.json');hand=load(OUT/'loaded_hand_dynamics/review.json')
    assert geom['all_passed'] and geom['all_offsets_requested'] and len(geom['cases'])==201
    assert hand['all_passed'] and hand['cube_disabled'] and len(hand['cases'])==9
    assert all(c['raw_max_self_mm']<=1e-6 and c['existing_palm7_seam_discarded_max_mm']==0 for c in hand['cases'])
    cases=[];sources=[]
    for front in (-15,-10,-5):
        path=OUT/f'loaded_front_{front}/review.json'
        if path.exists():
            r=load(path);assert r['candidate_sha256']==sha(OUT/'candidate.npz')
            cases.extend(r['cases']);sources.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path),cases=len(r['cases'])))
    complete={(c['book_front_mm'],c['gain_scale'],c['friction']) for c in cases}=={(f,g,mu) for f in (-15.,-10.,-5.) for g in (.5,1.,1.5) for mu in (.2,.3,.5)}
    failed=[dict(book_front_mm=c['book_front_mm'],gain_scale=c['gain_scale'],friction=c['friction'],stop=c['stop'],
        last_phase=c['last_phase'],external_cube_contact=c['external_cube_contact'],final_cube_supported_by_hand=c['final_cube_supported_by_hand']) for c in cases if not c['passed']]
    protected={
      'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
      'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
      'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
      'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d',
      'horizontal_quarter_motion_runner_20261027.py':'5c8487f134264f5ef4eb9f6bdf6a465e5fb6c69945c4a91407e1b6d75cc760cb'}
    assert all(sha(ROOT/p)==expected for p,expected in protected.items())
    searches=[load(ROOT/f'right_hand_rotation_20261029/{name}/review.json') for name in ('inverse_contact_search','inverse_contact_search_v2')]
    report=dict(status='observed_inverse54_research_pending_motor_connection',timestamp=dt.datetime.now().astimezone().isoformat(),
        latest_actual_review=ACTUAL.relative_to(ROOT).as_posix(),latest_actual_review_sha256=sha(ACTUAL),
        estimated_physical_placed_to_hold_deg=actual['estimated_palm_relative_placed_to_hold_deg'],
        estimated_physical_placed_to_open_deg=actual['estimated_palm_relative_placed_to_open_deg'],
        physical_inverse_90_verified=False,coarse_endpoint_hypotheses=sum(len(r['cases']) for r in searches),
        coarse_search_not_an_execution_certificate=True,whole_geometry_cases=201,whole_geometry_all_passed=True,
        finite_empty_hand_cases=9,finite_empty_hand_all_passed=True,loaded_conditions_available=len(cases),
        loaded_27_complete=complete,loaded_all_passed=bool(complete and not failed),loaded_failed_conditions=failed,
        loaded_sources=sources,unusable_larger_angle_candidates_preserved=True,
        latest_connection_review=PING.relative_to(ROOT).as_posix(),responding_motor_ids=[r['id'] for r in ping['motors'] if r['responded']],
        next_user_action='Confirm existing5V motor power; cube stays outside; no motion program yet.',
        next_user_command=None,ready_for_user_run=False,hardware_execution_allowed=False,
        assistant_motor_goal_writes=0,assistant_servo_setting_writes=0,protected_files_sha256=protected)
    (OUT/'research_progress.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    text=(
        '# 역회전 실물 확인과 후속 모델 검사\n\n'
        f"실물 영상에서는 배치에서 고정까지 약 {abs(report['estimated_physical_placed_to_hold_deg']):.1f}도, 손을 연 뒤 약 {abs(report['estimated_physical_placed_to_open_deg']):.1f}도 역회전한 것으로 추정된다. 90도 성공은 아직 확인하지 않았다.\n\n"
        '원속도 영상: [실물 잡기와 회전 및 OPEN](../../gapped_inverse_trial_20261029_184252_272166/offline_review/grasp_turn_open_realtime.mp4)\n\n'
        '목표만 더 늘린 후보는 손끝이 큐브와 떨어져 추가 회전이 거의 없었다. 재파지 일부는 기존 회전을 되돌렸다. 관절 목표를 탐색한 새 후보는 모델에서 약 60도에서 79도까지 회전했지만, 큰 각도 후보에는 손가락 접촉 또는 큐브 유지 실패가 있어 실행 대상으로 승인하지 않았다.\n\n'
        f"고정한 후속 후보의 전체 기하 검사 201조건과 큐브 없는 유한 구동 검사 9조건은 통과했다. 큐브 포함 검사는 현재 {len(cases)}조건이 기록됐으며, 실패 조건이 {len(failed)}개 있어 실물 큐브 시험은 준비되지 않았다. 빈손 통과가 큐브 유지나 90도 회전을 입증하지 않는다.\n\n"
        '마지막 연결 검사에서는 COM7이 열렸으나 16개 모터 모두 PING에 응답하지 않았다. 사용자에게 기존5V 전원 확인을 요청했다. 새 동작 명령은 제공하지 않았고, 도구로 모터 목표나 설정을 쓰지 않았다.\n'
    )
    (OUT/'연구_진행.md').write_text(text,encoding='utf-8')
    for rel in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        path=ROOT/rel;r=load(path);r.update(status=report['status'],current_status=report['status'],timestamp=report['timestamp'],
            next_command=None,next_user_command=None,next_user_command_ready=False,next_user_action_required=True,
            next_user_action=report['next_user_action'],ready_for_user_run=False,ready_for_supervised_operator_run=False,
            inverse_90_ready=False,inverse_research_progress=(OUT/'research_progress.json').relative_to(ROOT).as_posix(),
            current_motor_connection='COM7 opens; all16 individual PINGs unanswered',assistant_motor_or_setting_writes=0)
        path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    marker='<!-- inverse_research_20261029_connection_pending -->\n'
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig')
        if old.startswith(marker):old=old[old.index('<!-- end_inverse_research -->')+len('<!-- end_inverse_research -->'):].lstrip('\n')
        block=f"역회전 후속 연구: 실물 약54도 추정. 새 경로는 기하201/빈손유한9조건 통과. 큐브 포함 {len(cases)}조건 중 {len(failed)}조건 실패, 90도나 큐브 동작 준비로 승격하지 않는다. 현재 COM7은 열리지만 16모터 모두 무응답으로 기존5V 확인을 요청했다. 새 실행 명령은 아직 없다. 근거: right_hand_rotation_20261029/contact_retained_inverse/research_progress.json\n"
        path.write_text(marker+block+'<!-- end_inverse_research -->\n\n'+old,encoding='utf-8')
    print(json.dumps(dict(geometry=201,empty_finite=9,loaded_available=len(cases),loaded_complete=complete,failed=len(failed),
        responding_ids=report['responding_motor_ids'],ready=False,protected_sources_unchanged=True)))
if __name__=='__main__':main()
