"""Build a source-bound offline review without hiding failed conditions."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import hashlib,json,math,html,datetime
from pathlib import Path
import numpy as np
from review_common_grip_selection_20261105 import PROTECTED

R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip'
sources={}
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):
    sources[path.relative_to(R).as_posix()]=digest(path)
    return json.loads(path.read_bytes())

def main():
    candidate=O/'tip_squeeze/id1/plus12/candidate.npz';sha=digest(candidate);path=np.load(candidate);oldsource=R/'right_hand_rotation_20261104/forward_hold/candidate.npz';old=np.load(oldsource);forward_count=len(old['goals'])
    assert np.array_equal(path['start'],old['start']) and np.array_equal(path['goals'][:forward_count],old['goals'])
    maximum_step=int(np.max(abs(np.diff(np.vstack([path['start'],path['goals']]),axis=0))))
    assert maximum_step<=2
    protected={name:digest(R/name)==expected for name,expected in PROTECTED.items()}
    assert all(protected.values())
    certificate=read(O/'certificate_review_v3_thumb12.json');assert certificate['candidate_sha256']==sha and certificate['model_cases']==27
    for row in certificate['raw_reports']:
        p=R/row['path'];assert digest(p)==row['sha256'];read(p)
    empty=[]
    for front in (-15,-10,-5):
        d=read(O/f'empty_path_v3_thumb12/front{front}/review.json');assert d['candidate_sha256']==sha and len(d['cases'])==3;empty.extend(d['cases'])
    placements=[]
    for forward in (-2,0,2):
        d=read(O/f'placement_offsets_v3_thumb12/forward{forward:+d}/review.json');assert d['candidate_sha256']==sha and len(d['cases'])==9;placements.extend(d['cases'])
    accepted=[c for c in placements if c['task_passed']]
    centers=[c['continuous']['final_center_world_m'] for c in accepted]
    placement_summary=dict(cases=len(placements),task_passed=len(accepted),original_dynamic_safety_passed=sum(bool(c['continuous'] and c['continuous']['safety_passed']) for c in placements),forward_and_side_offsets_mm=[-2,0,2],yaw_offsets_deg=[-5,0,5],gain_scale=1.,friction=.3,book_front_mm=-10.,successful_model_center_spread_mm=max(math.dist(x,y)*1000 for x in centers for y in centers),failed_cases=[dict(forward_mm=c['forward_offset_mm'],side_mm=c['side_offset_mm'],yaw_deg=c['initial_yaw_offset_deg'],reasons=c['reasons']) for c in placements if not c['task_passed']],physical_placement_tolerance_verified=False)
    reactions=[]
    for p in sorted((O/'layer_reaction').glob('selected_v3_thumb12*/review.json')):
        d=read(p);assert d['candidate_sha256']==sha and len(d['cases'])==3
        raw=O/'full_placement_v3_thumb12/front-10/review.json';assert d['source_matrix_sha256']==digest(raw)
        state=R/d['source'];assert d['source_sha256']==digest(state)
        for c in d['cases']:reactions.append(dict(gain_scale=d['gain_scale'],friction=d['friction'],torque_nm=c['hypothetical_reaction_torque_nm'],safety_passed=c['safety_passed'],task_passed=c['load_filter_passed'],yaw_change_deg=c['max_yaw_change_deg'],shift_mm=c['max_center_shift_mm'],reasons=c['reasons']))
    assert len(reactions)==27
    loaded=[c for c in reactions if c['torque_nm']!=0];zero=[c for c in reactions if c['torque_nm']==0]
    reaction_summary=dict(all_cases=len(reactions),original_dynamic_safety_passed=sum(c['safety_passed'] for c in reactions),loaded_cases=len(loaded),loaded_filter_passed=sum(c['task_passed'] for c in loaded),nominal_gain_loaded_passed=sum(c['task_passed'] for c in loaded if c['gain_scale']==1.),nominal_gain_loaded_cases=sum(c['gain_scale']==1. for c in loaded),zero_load_continuation_passed=sum(c['task_passed'] for c in zero),zero_load_continuation_cases=len(zero),assumed_torque_nm=[-.01,.01],failed_cases=[c for c in reactions if not c['task_passed']],actual_left_hand_load_measured=False,rigid_whole_cube_reaction_only=True,articulated_layer_not_simulated=True)
    seams=[]
    for p in sorted((O/'next_rotation').glob('*/review.json')):
        d=read(p)
        seams.append(dict(source=p.relative_to(R).as_posix(),candidate_sha256=d['candidate_sha256'],order=d.get('order','simultaneous'),safety_passed=d['dynamic']['safety_passed'],completed=d['dynamic']['completed_goals']==d['dynamic']['total_goals'],additional_yaw_deg=d['additional_yaw_deg'],connection_filter_passed=d['diagnostic_connection_filter_passed'],reasons=d['grip_task_reasons']))
    extensions=[]
    for p in sorted((O/'yaw90_extension').glob('*/review.json')):
        d=read(p);extensions.append(dict(source=p.relative_to(R).as_posix(),translation_allowed=d.get('temporary_translation_allowed_in_proposal',False),proposal_failed=d['proposal_failed'],free_cube_replayed=d['free_cube_result'] is not None,whole_turn_target90_passed=d.get('whole_turn_target90_passed',False)))
    free_search=[]
    for p in sorted((O/'free_extension_search').glob('*/review.json')):
        d=read(p);best=d['cases'][d['best_index']];free_search.append(dict(source=p.relative_to(R).as_posix(),evaluations=len(d['cases']),safe=sum(c['dynamic']['safety_passed'] for c in d['cases']),best_yaw_deg=best['whole_yaw_deg'],target90_filter_passed=d['best_model_target90_passed'],complete_initial_placement_replayed=False))
    contract=read(O/'integration_orientation_routes.json')
    sources[(O/'two_hand_interface_draft.md').relative_to(R).as_posix()]=digest(O/'two_hand_interface_draft.md')
    sources[(O/'regrasp_constraints_20261106.md').relative_to(R).as_posix()]=digest(O/'regrasp_constraints_20261106.md')
    photos=read(O/'model_discrepancy/saved_marker_rotation/review.json');phase=read(O/'model_discrepancy/phase_yaw_comparison.json');diagnostic=read(O/'model_discrepancy/recorded_joints_f0.3/review.json');software=read(O/'software_checks.json');handshake=read(O/'integration_dry_run_v3.json');media=read(O/'evidence_thumb12/manifest.json')
    assert software['successful'] and handshake['candidate_sha256']==sha and media['candidate_sha256']==sha
    assert digest(R/media['source_review'])==media['source_review_sha256']
    for module,expected in software['test_source_sha256'].items():
        assert digest(R/module)==expected;sources[module]=expected
    followup=read(O/'offline_followup.json')
    for p,h in followup['source_documents_sha256'].items():
        assert digest(R/p)==h,p
        if (R/p).suffix=='.json':read(R/p)
        else:sources[p]=h
    layer_media=read(O/'articulated_upper_layer/evidence/manifest.json')
    assert digest(R/layer_media['source_review'])==layer_media['source_review_sha256']
    near_media=read(O/'near_quarter_full_replay/evidence/manifest.json')
    assert digest(R/near_media['source_review'])==near_media['source_review_sha256']
    for video in near_media['videos']:
        p=O/'near_quarter_full_replay/evidence'/video['file'];assert digest(p)==video['sha256'];sources[p.relative_to(R).as_posix()]=digest(p)
    implementation=['canonical_grip_dynamics_20261105.py','upper_layer_separation_certificate_20261105.py','canonical_grip_task_review_20261105.py','offline_two_hand_coordinator_20261105.py','evaluate_canonical_grip_reaction_20261105.py','screen_canonical_grip_empty_20261105.py','screen_canonical_grip_placement_offsets_20261105.py','render_canonical_grip_evidence_20261105.py','measure_saved_forward_marker_rotation_20261105.py','diagnose_recorded_forward_kinematics_20261105.py','cube_task_frame_20261105.py','offline_fractional_goal_accumulator_20261105.py','review_common_grip_cube_symmetry_20261105.py','review_common_grip_face_holds_20261105.py','validate_green_grip_default_regression_20261105.py','analyze_saved_forward_rotation_uncertainty_20261105.py','research_common_grip_rolling_yaw_20261105.py','propose_sliding_workspace_grip_20261105.py','evaluate_sliding_grip_transition_20261105.py','research_workspace_lift_yaw_20261105.py','review_wide_initial_yaw_20261105.py','build_articulated_upper_layer_20261105.py','evaluate_articulated_upper_layer_20261105.py','validate_articulated_upper_layer_20261105.py','render_articulated_upper_layer_20261105.py','summarize_offline_followup_20261105.py']
    implementation.extend(['audit_common_grip_contact_wrench_20261105.py','inspect_simulated_contact_dimensions_20261105.py','search_index_assisted_yaw_20261105.py','screen_common_grip_torque_caps_20261105.py','probe_torque_capped_grip_load_20261105.py','research_quasistatic_wrench_path_20261105.py','native_pyramidal_point_forces_20261106.py','checkpoint_quarter_review_20261106.py','diagnose_wrench_path_stop_20261106.py','research_postturn_regrasp_20261106.py','replay_near_quarter_full_20261106.py','render_near_quarter_evidence_20261106.py','probe_postturn_clearance_20261106.py'])
    implementation.extend(['inspect_postturn_workspace_20261106.py','search_postturn_thumb_clearance_20261106.py','research_postturn_thumb_ik_20261106.py'])
    implementation.append('evaluate_palm_supported_quarter_load_20261106.py')
    implementation.extend(['offline_goal_tracking_bounds_20261106.py','search_postturn_thumb_workspace_20261106.py','inspect_thumb_workspace_20261106.py','inspect_postturn_contact_geometry_20261106.py','research_countergrip_level_20261106.py','inspect_countergrip_level_20261106.py','research_middle_support_transfer_20261106.py','research_initial_station_replay_20261106.py','research_thumb_corner_support_20261106.py','research_lower_countergrip_station_20261106.py','evaluate_lower_countergrip_station_20261106.py','render_countergrip_progress_20261106.py'])
    implementation.append('inspect_countergrip_reports_20261106.py')
    implementation.extend(['search_countergrip_fixed_prefix_20261106.py','research_middle_side_brace_20261106.py','research_pinch_transfer_turn_20261106.py','probe_temporary_handoff_load_20261106.py','temporal_two_hand_coordinator_20261106.py','build_temporal_handoff_review_20261106.py','inspect_rollout_runtime_20261106.py'])
    implementation.extend(['build_canonical_grip_offline_packet_20261105.py','validate_canonical_grip_offline_packet_20261105.py','run_canonical_grip_software_checks_20261105.py'])
    # Operator text is Korean; statuses above remain machine-readable English.
    remaining=[
        '새 경로는 처음 배치부터 약88.84도와10초 고정까지 동적 기준을 통과했다. 회전 뒤 윗단 공간을 비우고 하단을 다시 잡는 경로는 아직 미통과다.',
        '회전 후 공통 파지와 위치를 정하고 다음 회전으로 연결한다. 공통 위치 복귀와 반복 연결은 아직 미완성이다.',
        '실물과 모델의 초기 큐브 위치 및 접촉을 비교하고 새 파지의 윗단 공간과 고정력을 확인한다.',
        '두 손을 합친 뒤 실제 상단 회전 하중, 왼손 접근과 퇴각, 프레임 간섭을 확인한다.',
    ]
    report=dict(status='offline_research_packet_live_execution_not_ready',created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),current_offline_candidate=candidate.relative_to(R).as_posix(),candidate_sha256=sha,selection='preferred model candidate for later physical review; broad-load robustness remains incomplete',retained_forward_source=oldsource.relative_to(R).as_posix(),retained_forward_source_sha256=digest(oldsource),original_forward_goals_unchanged=forward_count,total_goals=len(path['goals']),added_regrasp_and_hold_goals=len(path['goals'])-forward_count,nominal_seconds=len(path['goals'])*.075,maximum_goal_increment_ticks=maximum_step,full_model_matrix=dict(cases=27,original_dynamic_safety_passed=certificate['original_dynamic_safety_passed'],task_passed=certificate['certificate_based_task_passed'],legacy_clipped_gap_filter_also_passed=certificate['legacy_task_passed'],scope='3 support projections x3 actuator-gain hypotheses x3 friction hypotheses; not27 independent physical placements',regrasp_yaw_range_deg=certificate['regrasp_yaw_range_deg'],tilt_range_deg=certificate['tilt_range_deg'],certified_gap_range_mm=certificate['certified_gap_lower_bound_range_mm'],parameter_uncertainty_center_spread_mm=certificate['model_parameter_terminal_center_spread_mm']),hand_only=dict(cases=len(empty),passed=sum(c['passed'] for c in empty),maximum_tracking_rad=max(c['worst']['tracking_rad'] for c in empty),current_actual_OFF_recovery_not_reviewed=True),initial_placement_sensitivity=placement_summary,hypothetical_reaction=reaction_summary,next_rotation_connections=seams,additional90deg_extension_proposals=extensions,photographic_phase_rotation_estimate=phase,recorded_joint_diagnostic_is_not_a_path_validation=True,recorded_joint_diagnostic=dict(changed_hand_inertia=True,pose_registration_unverified=True,actual_joint_replay_final_yaw_deg=diagnostic['cases'][-1]['final_yaw_deg'],actual_joint_replay_maximum_cube_penetration_mm=diagnostic['cases'][-1]['worst']['cube_mm']),software_tests=dict(run=software['tests_run'],successful=software['successful']),integration=dict(synthetic_intents_only=True,transport_adapters=0,next_cycle_seam_blocked=True,physical_target_orientation_required=True),remaining=remaining,physical_target90_verified=False,new_physical_grip_verified=False,actual_left_hand_load_verified=False,hardware_execution_allowed=False,current_user_physical_action_required=False,assistant_motor_writes=0,devices_opened=False,protected_sources_unchanged=protected,source_documents_sha256=sources,implementation_sha256={p:digest(R/p) for p in implementation})
    report['free_cube_additional_yaw_search']=free_search
    report['idealized_orientation_states']=contract['logical_orientation_states']
    report['face_hold_hypotheses']=followup['face_hold_hypotheses']
    report['cube_label_symmetry_static_only']=followup['cube_label_symmetry']
    report['default_green_regression']=followup['unchanged_default_green_regression']
    report['articulated_upper_layer_hypotheses']=followup['articulated_upper_layer']
    for key in ('near_quarter_full_initial_replay','torque_capped_full_path','torque_capped_load_hypotheses','native_static_contact_force_audit','motor_model_force_scope','contact_translation_control_directions'):report[key]=followup[key]
    report['additional_research_all_failures_retained']=followup['research']
    report['new_valid_whole_cube90_path']=False
    grip_progress=read(O/'countergrip_level/seed0_source1_lift2_normal1.5_n850/review.json');grip_replay=grip_progress['recorded_goal_replay']
    assert grip_replay['safety_passed'] and not grip_progress['checkpoint90_and_new_grip_passed']
    report['postturn_partial_progress']=dict(yaw_deg=grip_replay['final_yaw_from_placement_deg'],tilt_deg=grip_replay['final_up_tilt_deg'],opposing_lower_contacts='index and thumb',contact_fraction=grip_replay['settled_tail']['opposed_lower_body_contact_fraction'],fixed_goal_hold_seconds=grip_replay['full_fixed_hold']['seconds'],complete=False,reasons=grip_progress['reasons'],remaining_obstruction='index and thumb overlap the upper-layer sweep',evidence='countergrip_progress/index.html',checkpoint_continuation_not_full_initial_replay=True)
    report['temporal_two_hand_protocol']=read(O/'temporal_handoff_review.json')
    (O/'offline_review.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    failrows=''.join('<tr><td>'+str(c['gain_scale'])+'</td><td>'+str(c['friction'])+'</td><td>'+str(c['torque_nm'])+'</td><td>'+', '.join(c['reasons'])+'</td></tr>' for c in reaction_summary['failed_cases'])
    rows=[('큐브를 자유롭게 둔 전체 경로','27 / 27','받침3종, 구동 배율3종, 마찰3종'),('손만 움직인 경로','9 / 9','현재 실제 OFF 자세에서의 복귀는 별도'),('초기 배치 오차','25 / 27','앞뒤와 좌우 ±2mm, 방향 ±5도인 모델 가설'),('가정한 반력 ±0.01N·m',f"{reaction_summary['loaded_filter_passed']} / {len(loaded)}",'기준 구동6/6. 약한 구동 조건의 실패 포함'),('기존 저장 사진의 회전','전체 약80도 / 회전 구간 약60도','미보정 마커 치수와 카메라 기준 추정'),('새 파지에서 다음 회전 연결','미통과','목표를 이어 붙이는 방식은 채택하지 않음')]
    rows.extend([('선택한 면을 위로 유지','6 / 6','각 면을 처음부터 따로 배치한10초 정지 시험. 뒤집는 경로는 아님'),('분리된 윗단 회전 하중','6 / 6','가정한 마찰3종과 좌우90도. 실제 왼손 모양과 접촉은 없음')])
    near=followup['near_quarter_full_initial_replay'];caps=followup['torque_capped_full_path'];capped_load=followup['torque_capped_load_hypotheses']
    rows.extend([('새 전체 회전 경로',f"{near['whole_cube_yaw_deg']:.2f}도 +10.05초 고정",'초기 배치부터2628목표 재생. 회전 뒤 공통 파지는 미통과'),('더 낮은 모델 관절 토크 한도',f"{caps['task_passed']} / {caps['cases']}",'0.15와0.23N·m 가설. 실제450mA 환산값이 아님'),('토크 한도가 있는 모델의 반력',f"{capped_load['nonzero_load_passed']} / {capped_load['nonzero_load_cases']}",'기준 구동과 마찰에서 ±0.01N·m인4조건. 별도 무부하2조건 포함 시6/6')])
    table=''.join('<tr><th>'+html.escape(a)+'</th><td>'+html.escape(b)+'</td><td>'+html.escape(c)+'</td></tr>' for a,b,c in rows)
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>오른손 회전과 공통 파지 검토</title><style>body{font:16px/1.65 system-ui,sans-serif;color:#24313c;background:#f3f5f7;max-width:1120px;margin:30px auto;padding:0 20px}h1{font-size:27px}h2{font-size:21px;margin-top:30px}.box,section{background:white;border-radius:12px;padding:18px;margin:15px 0}.pending{background:#fff0ce}.flow{display:flex;gap:10px;flex-wrap:wrap}.flow span{background:#e5edf7;border-radius:8px;padding:10px}.flow .todo{background:#eee}table{border-collapse:collapse;width:100%;background:white}td,th{border-bottom:1px solid #dce1e5;text-align:left;padding:11px}th{font-weight:600}video{width:100%;max-width:960px}img{width:100%;max-width:960px}.small{color:#596775;font-size:14px}a{color:#145c9e}details{background:white;padding:15px;border-radius:8px;margin-top:15px}</style><h1>오른손 회전 후 공통 파지 검토</h1><p class="box pending">목표는 큐브를 원하는 방향으로 90도 돌리고, 다른 손이 윗단을 돌리는 동안 하단을 같은 자리에서 잡는 것이다. 현재 90도 회전과 연속 연결은 완성되지 않았다. 실물 장치를 연결하지 않고 아래 결과를 준비했다.</p><div class="flow"><span>배치</span><span>방향 변경<br>실물 영상 확보</span><span>하단 재파지<br>새 모델 후보</span><span class="todo">다른 손의 윗단 회전<br>통합 확인 예정</span><span class="todo">다음 방향 변경<br>연결 경로 개발 중</span></div><h2>현재 결과</h2><table>'''+table+'''</table><p class="small">모델 시험 횟수는 실물 성공률이 아니다. 실패 조건과 미확인 항목도 아래에 남겼다. 새 후보는 기존 회전 목표1417개를 보존하고, 재파지와 유지327개를 뒤에 추가했다.</p><section><h2>재파지와 유지 영상</h2><p>파랑은 검지, 초록은 중지, 노랑은 약지, 빨강은 엄지다. 주황 원통은 윗단이 회전할 공간이다. 모델의 기존 회전은 약61도다.</p><video controls preload="metadata" src="evidence_thumb12/regrasp_and_hold_1x.mp4"></video><p><a href="evidence_thumb12/index.html">전체 경로 4배속과 여러 방향의 그림 보기</a></p></section><section><h2>마지막 파지</h2><img src="evidence_thumb12/new_nominal_side.png" alt="윗단 회전 공간 아래에서 큐브를 받치는 새 모델 파지"><p>엄지 끝관절의 목표를 이전 후보보다12틱, 약1.05도 조정했다. 전류나 PID를 높이지 않았다. 마지막4.05초의 모든1ms 모델 상태에서 윗단 공간의 최소 분리 여유는 전체 조건 중 약0.83mm다.</p></section><details><summary>반력 시험의 실패 조건</summary><p>전체 큐브에 가정한 반력을 가했다. 실제로 분리된 윗단과 왼손을 시뮬레이션한 결과는 아니다. 약한 구동과 낮은 마찰 한 조건에서는 큐브가 이탈해 기존 동적 기준을 넘었다. 나머지 실패에는 방향 흔들림, 이동, 공간 여유 또는 접촉 부족이 포함된다.</p><table><tr><th>모델 구동 배율</th><th>마찰</th><th>가정 반력 N·m</th><th>미달 항목</th></tr>'''+failrows+'''</table></details><details><summary>배치 오차와 위치 재현성</summary><p>앞뒤−2mm와 좌우−2mm가 겹친 배치 중 방향−5도와0도 두 조건은 하단의 양쪽 접촉이 부족했다. 통과한25조건의 최종 중심 분산은 같은 모델에서 '''+f"{placement_summary['successful_model_center_spread_mm']:.2f}"+'''mm였다. 구동력과 마찰 가설까지 바꾸면 중심 범위가 약11.54mm로 커진다. 실물의 위치 반복 정밀도를 측정한 값은 아니다.</p></details><section><h2>남은 작업</h2><ol>'''+''.join('<li>'+html.escape(v)+'</li>' for v in remaining)+'''</ol><p>지금 추가 촬영이나 기기 연결을 할 필요는 없다. 새 실물 실행 명령은 만들지 않았다. 다시 실물을 검토할 때에는 현재 빈손 관절값부터 확인한다.</p></section><p><a href="offline_review.json">전체 결과와 출처 SHA256</a> / <a href="certificate_review_v3_thumb12.json">전체 경로 상세</a> / <a href="integration_dry_run_v3.json">양손 순서의 기기 없는 모의 실행</a> / <a href="software_checks.log">소프트웨어 시험 기록</a></p></html>'''
    page=page.replace('<a href="software_checks.log">소프트웨어 시험 기록</a>', '<a href="software_checks.log">소프트웨어 시험 기록</a> / <a href="two_hand_interface_draft.md">양손 연결 인터페이스 초안</a>')
    layer=followup['articulated_upper_layer']
    layer_section='<section><h2>윗단만 회전하는 별도 모델</h2><p>오른손의 관절 목표를 고정하고 분리된 윗단에 가정한 외부 회전 힘을 주었다. 아래 몸통의 최대 회전은 '+f"{layer['maximum_lower_body_yaw_deg']:.2f}"+'도, 최대 이동은 '+f"{layer['maximum_lower_body_shift_mm']:.2f}"+'mm였다. 실제 왼손과 큐브 내부 기구는 단순화되어 있다. 오른손이 큐브 전체를90도 돌리는 경로의 완성과는 별개다.</p><video controls preload="metadata" src="articulated_upper_layer/evidence/upper_layer_plus90_1x.mp4"></video><p><a href="articulated_upper_layer/evidence/index.html">영상 설명과 전후 그림</a> / <a href="offline_followup.json">추가 탐색과 실패 결과</a></p></section>'
    page=page.replace('<section><h2>남은 작업</h2>',layer_section+'<section><h2>남은 작업</h2>')
    near_section='<section><h2>새 경로: 약89도 회전 후 고정</h2><p>처음 큐브를 놓은 상태부터 자유 큐브를 재생해 '+f"{near['whole_cube_yaw_deg']:.2f}"+'도 회전과10.05초 고정까지 기존 동적 기준을 통과했다. 고정 구간의 방향 변화는 '+f"{near['fixed_hold_yaw_drift_deg']:.3f}"+'도, 중심 이동은 '+f"{near['fixed_hold_shift_mm']:.3f}"+'mm였다. 마지막 자세는 손바닥과 검지 쪽으로 받쳐진 상태다. 윗단 공간 침범과 하단 접촉 부족이 남아 전체 목표 완료로 세지 않는다.</p><video controls preload="metadata" src="near_quarter_full_replay/evidence/whole_path_6x.mp4"></video><p><a href="near_quarter_full_replay/evidence/index.html">추가 회전 원래 속도 영상과 전후 그림</a> / <a href="near_quarter_full_replay/review.json">전체 재생 결과와 미달 항목</a></p></section>'
    force_section='<details><summary>모델 구동력과 접촉 계산의 범위</summary><p>기존 모델은 관절 토크 제한이 활성화되지 않아, 별도로0.15와0.23N·m의 더 낮은 한도를 적용해 기존 전체 경로를 재검사했다. 두 기준 조건은 통과했다. 이 값은 실제450mA를 변환한 값이 아니다. <a href="https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#goal-current102">제조사 전류 설명</a>을 확인했으며 실물 전류와 관절 토크의 보정은 아직 필요하다. 접촉 계산은 모델의 원래 피라미드 마찰 조건을 사용한다. 정적인 힘 균형8조건 통과를 회전 경로 성공으로 세지 않는다.</p><p>새 경로는 정확한 모델 자세와 접촉 정보를 사용해 만든 관절 목표다. 실물 카메라 기반 정책이나 SAC 실행이 아니다. 실제 모터 설정과 구동 파일은 바꾸지 않았다.</p></details>'
    page=page.replace('<section><h2>재파지와 유지 영상</h2>',near_section+force_section+'<section><h2>재파지와 유지 영상</h2>')
    thumb=followup['research']['postturn_thumb_geometry']
    planning_section='<details><summary>회전 뒤 다시 잡기: 진행 중인 작업</summary><p>윗단 공간과 가장 가까운 부분은 엄지 끝 형상이다. 중지를 기존 자세로 돌린 뒤에도 엄지 끝이 공간에 남았다. 엄지의 작은 보정 '+str(thumb['cases'])+'조건을 검사했으며, 공간을 비운 조건에서는 큐브를 누르는 엄지 접촉이 사라졌다. 공간과 하단 접촉을 동시에 만족하는 조합은 아직 통과하지 못했다. 정지해 있다는 이유만으로 다른 손의 회전 하중을 버틴다고 표시하지 않는다.</p></details>'
    page=page.replace('<section><h2>남은 작업</h2>',planning_section+'<section><h2>남은 작업</h2>')
    load_rows=''.join('<tr><td>'+f"{g['downward_N']:g}"+'</td><td>'+str(g['functional_probe_passed'])+' / '+str(g['cases'])+'</td><td>'+', '.join(f"{c['torque_Nm']:+g}" for c in g['results'])+'</td></tr>' for g in followup['research']['palm_supported_functional_load']['groups'])
    functional_section='<details><summary>손바닥과 검지만으로 받치는 대안의 하중 시험</summary><p>엄지를 펴서 윗단 공간을 비운 상태에 가정한 회전 하중을 가했다. 엄지와 중지가 반드시 접촉해야 한다는 기존 파지 판정은 그대로 남기고, 실제 방향 흔들림과 이동, 윗단 여유를 유지하는 기능 시험을 따로 기록했다. 아래 누름 힘은 다른 손이 제공한다고 가정한 값이며 실제 하드웨어 능력은 확인하지 않았다.</p><table><tr><th>가정한 아래 방향 힘 N</th><th>기능 시험 통과</th><th>시험한 회전 하중 N·m</th></tr>'+load_rows+'</table><p>추가 누름 힘이 없으면 ±0.003N·m는 버텼지만 ±0.01N·m에서 미끄러졌다. 정적 받침이나 작은 하중 통과를 완전한 파지나 실물 통합 성공으로 세지 않는다.</p></details>'
    page=page.replace('<section><h2>남은 작업</h2>',functional_section+'<section><h2>남은 작업</h2>')
    partial_section='<section><h2>회전 뒤 재파지의 진행 상황</h2><p>회전 뒤 검지와 엄지로 다시 잡고 수평을 맞춘 경로는 약89.19도, 기울기 약0.76도에서10.05초 고정을 통과했다. 하단 접촉은 유지됐지만 두 손가락이 윗단 회전 공간을 침범해 완성으로 세지 않는다. 엄지를 먼저 낮추면 약89.31도를 유지했으나 검지가 남았고, 둘 다 낮추면 약83.58도로 되돌아왔다.</p><p><a href="countergrip_progress/index.html">실패 지점까지 포함한 새 영상과 비교 그림 보기</a></p><p>큐브 밑모서리 받침, 중지의 받침 인계, 초기 배치 변경도 비교했다. 정적인 그림의 통과와 실제 자유 큐브 이동의 통과는 따로 기록했다. 새 경로는 아직 실물 실행 대상으로 채택하지 않았다.</p></section>'
    page=page.replace('<h2>현재 결과</h2>',partial_section+'<h2>현재 결과</h2>')
    page=page.replace('<h2>현재 결과</h2>','<section><h2>윗단 공간을 비우는 시점</h2><p>오른손 전체 회전과 재파지 중에는 공간을 사용한다. 다른 손의 접근과 윗단 회전, 퇴각이 끝날 때까지는 공간 확보와 하단 고정을 유지한다. 회전 완료와 접근 허가를 별도 단계로 나눈 오프라인 순서를 구현했다. 실제 물리 경로의 연속 연결은 아직 미완성이다.</p><p><a href="temporal_handoff_review.html">단계별 공간 사용과 후보 거부 기록 보기</a></p></section><h2>현재 결과</h2>')
    (O/'offline_review.html').write_text(page,encoding='utf-8')
    media_page=O/'evidence_thumb12/index.html'
    evidence_text=media_page.read_text(encoding='utf-8').replace('이 모델의 기존 회전은 약61도로, 실물 영상에서 관찰한 약90도와 차이가 있다.','이 모델의 회전 구간은 약61도다. 저장된 실물 사진 추정은 초기 파지를 포함해 약80도, 파지 후 회전 구간은 약60도이며 정확한90도 완료는 미확인이다.')
    media_page.write_text(evidence_text,encoding='utf-8')
    note='''# 현재 오프라인 검토 상태

우선 후보는 `tip_squeeze/id1/plus12/candidate.npz`다. 전체 모델27조건과 손만9조건은 통과했으며 배치 오차는25/27이다. 기준 구동의 가정 반력 ±0.01N·m는6/6이나 전체 구동 가설에서는14/18로, 약한 구동과 낮은 마찰에서 실패가 남는다.

사진으로 다시 계산한 두 번째 실물 실행은 전체 약80도, 파지 후 회전 구간 약60도의 추정값이다. 모델은 파지 중 거의 회전하지 않고 회전 구간에서약61도다. 정확한 실물90도 완료로 승격하지 않는다. 초기 배치와 접촉의 정합을 검토한다.

공통 파지에서 다음 회전을 단순 연결한 안은 미통과다. 중지부터 움직인 안은 동적 경로를 끝냈지만 큐브가 반대로약38도 돌아가고 하단 접촉을 잃었다. 기존 배치용 경로를 그대로 반복 사용하지 않는다.

90도까지 추가 회전하는 고정 접촉 IK 제안도 현재 미통과다. 큐브 위치를 잠근 IK 그림은 자유 큐브의 회전 성공으로 세지 않는다. 기기를 연결하거나 실제 동작을 시키지 않는다.

현재 설명과 영상은 `offline_review.html`, 출처를 묶은 결과는 `offline_review.json`이다. 실물 새 파지, 현재 OFF 복귀, 실제 왼손 하중과 진입 간섭은 추후 확인 항목이다.
'''
    note+='\n선택한 윗면을 기준으로 여섯 면의 개별10초 정지 파지는6/6이다.24개 큐브 방향 대칭은 같은 점유 형상의 정적 검사이며24개 회전 경로가 아니다. 윗단을 분리한 별도 모델은 마찰3종과 방향2종의6조건에서 통과했으나 실제 왼손과 내부 큐브 기구는 아직 단순화된 가설이다.\n\n추가 전체90도 탐색은 `offline_followup.json`에 실패를 포함해 정리했다. 접촉점을 유지한 채 관절을 재배치하는 경로가 여전히 필요하다. 새 실물 실행 지시는 만들지 않았다.\n'
    note+='\n2026-11-06 추가: `near_quarter_full_replay`에서 처음 배치부터2628목표를 재생해 약88.84도와10.05초 고정을 동적 기준 내에서 확인했다. 마지막 큐브는 손바닥과 검지 쪽으로 받쳐져 있다. 공통 중심 복귀, 전체 목표 자세, 윗단 공간, 요구한 하단 접촉은 미달이며 현재 우선 공통 파지 후보를 교체하지 않는다. 다음 작업은 이 회전 뒤 재파지 경로다.\n\n0.15와0.23N·m 모델 토크 한도 가설은 기존 전체 경로2/2, 기준 조건의 비영 반력4/4를 통과했다. 실제450mA의 토크 환산은 아니며 실물 설정은 바꾸지 않았다.\n'
    (O/'next_stage.md').write_text(note,encoding='utf-8')
    with (O/'next_stage.md').open('a',encoding='utf-8') as f:
        f.write('\n2026-11-06 재파지 추가: 검지와 엄지의 대향 접촉으로 약89.19도, 기울기0.76도에서10.05초 고정까지 진행했다. 윗단 공간은 미달이다. 엄지 낮추기는89.31도를 유지하지만 검지가 공간에 남고, 양쪽을 내리면83.58도로 되돌아온다. 정적 IK 통과를 자유 큐브 경로 성공으로 표시하지 않는다. 비교 영상: `countergrip_progress/index.html`.\n')
    # Preserve historical summary data, but prevent old v1 counts being read as current.
    summary=O/'offline_summary.json';old_summary=json.loads(summary.read_bytes());old_summary['superseded_by']='offline_review.json';old_summary['historical_summary_not_latest_candidate']=True;summary.write_text(json.dumps(old_summary,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    progress=R/'professor_progress_20261015.md';data=progress.read_bytes();begin=b'<!-- offline_packet_20261105_current -->';end=b'<!-- offline_packet_20261105_end -->'
    if data.startswith(begin):data=data.split(end,1)[1].lstrip(b'\r\n')
    elif data.startswith(b'## CURRENT '):data=data.replace(b'## CURRENT ',b'## PREVIOUS ',1)
    header='''<!-- offline_packet_20261105_current -->
## CURRENT 2026-11-05: 오프라인 파지 후보 검토와 회전량 재확인

- 기기를 열지 않았다. 사용자 요청에 따라 손과 웹캠 없이 계산했고 새 실물 실행 명령은 만들지 않았다.
- 우선 모델 후보는 엄지 끝 목표12틱을 조정한 `tip_squeeze/id1/plus12/candidate.npz`다. 원래1417목표는 보존하고 뒤에327목표를 추가했다. 전체 모델27/27, 손만9/9, 배치 오차25/27이다.
- 가정 반력 ±0.01N·m는 기준 구동6/6, 전체 구동 가설14/18이다. 약한 구동과 낮은 마찰에서 큐브 이탈 등 실패가 있어 전 조건 하중 저항으로 표시하지 않는다. 실제 왼손 하중은 미측정이다.
- 저장 사진의 두 면 마커로 계산한 두 번째 실행은 전체 약79.9도, 파지 이후 회전 구간 약60.2도의 추정이다. 모델의 회전 구간약61.4도와 비슷하다. 마커 부착과 카메라 움직임이 별도 보정되지 않아 정확한 실물90도 인증은 아니다. 초기 파지 때 실물에서 추가 회전이 생긴 차이를 확인했다.
- 공통 파지에서 다음 회전으로 이어지는 연결은 미통과다. 중지 우선 안은 동적 재생을 끝냈지만 큐브가 반대로약38도 움직여 채택하지 않았다. 추가28.75도 IK도 현재 미통과로, 성공으로 세지 않는다.
- 소프트웨어 시험50개 통과. 양손 통합은 전송 어댑터 없는 모의 순서만 준비했다. 실제 목표 방향 확인이 필요하며 배치용 경로를 다음 회전의 입력으로 재사용하지 않는다.
- 현재 문서와 영상: `right_hand_rotation_20261105/canonical_body_grip/offline_review.html`. 결과와 모든 출처 해시는 같은 폴더 `offline_review.json`이다. 보호 대상6개 파일은 그대로다.

<!-- offline_packet_20261105_end -->

'''.replace('\n','\r\n').encode('utf-8')
    header=header.replace('소프트웨어 시험50개'.encode('utf-8'),f"소프트웨어 시험{software['tests_run']}개".encode('utf-8'))
    header=header.replace(b'CURRENT 2026-11-05',b'CURRENT 2026-11-06')
    grip_note='- 재파지 추가: 검지와 엄지의 대향 접촉으로 약89.19도, 기울기0.76도에서10.05초 고정까지 진행했다. 윗단 공간 미달로 채택하지 않았다. 양쪽을 낮추면83.58도로 돌아오는 문제가 남는다. 새 영상은 `countergrip_progress/index.html`에 있다.\r\n'
    header=header.replace(b'\r\n<!-- offline_packet_20261105_end -->',grip_note.encode('utf-8')+b'\r\n<!-- offline_packet_20261105_end -->')
    near_note=f"- 새 전체 경로는 초기 배치부터2628목표를 재생해 {near['whole_cube_yaw_deg']:.2f}도 회전과10.05초 고정까지 동적 기준을 통과했다. 회전 뒤 윗단 공간과 하단 재파지, 공통 위치 복귀는 미통과다. 영상은 `near_quarter_full_replay/evidence/index.html`에 있다.\r\n- 모델 토크 한도0.15와0.23N·m 가설에서 기존 전체 경로2/2, 기준 구동과 마찰의 비영 반력4/4를 통과했다. 실제450mA의 토크 환산이나 실물 설정 변경은 아니다.\r\n"
    header=header.replace(b'\r\n<!-- offline_packet_20261105_end -->',near_note.encode('utf-8')+b'\r\n<!-- offline_packet_20261105_end -->')
    addition=f"- 추가 오프라인 검토: 여섯 면의 개별10초 정지 파지6/6,24개 방향 대칭은 정적 검사다. 윗단을 분리한 하중 모델은 마찰3종과 좌우 회전의6조건에서 통과했으며 아래 몸통의 최대 회전{layer['maximum_lower_body_yaw_deg']:.2f}도, 이동{layer['maximum_lower_body_shift_mm']:.2f}mm였다. 실제 왼손과 큐브 내부 기구는 단순화한 가설이며 전체 큐브90도 경로 완성과는 별개다.\r\n- 들어올리기, 구르는 접촉, 큰 초기 방향 변경과 관절 여유를 늘리는 재파지 탐색은 아직 전체90도 경로로 통과하지 못했다. 실패 기록은 `offline_followup.json`에 출처와 함께 남겼다.\r\n"
    header=header.replace(b'\r\n<!-- offline_packet_20261105_end -->',addition.encode('utf-8')+b'\r\n<!-- offline_packet_20261105_end -->')
    progress.write_bytes(header+data)
    print(json.dumps(dict(candidate_sha256=sha,model_matrix=27,empty=sum(c['passed'] for c in empty),placement_pass=len(accepted),reaction_loaded_pass=reaction_summary['loaded_filter_passed'],reaction_loaded_cases=len(loaded),software_tests=software['tests_run'],source_documents=len(sources),protected_unchanged=all(protected.values()),hardware_execution_allowed=False),ensure_ascii=False),flush=True)

if __name__=='__main__':
    main()
    from finalize_nominal_handoff_packet_20261106 import main as finalize_current_nominal
    finalize_current_nominal()
