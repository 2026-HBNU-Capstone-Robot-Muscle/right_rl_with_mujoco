"""Publish completed native results separately from the earlier preparation record."""
from pathlib import Path
import hashlib
import json
import re
import html
import numpy as np

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
BASE=OUT/'next_cycle/handoff_refinement_20261110';FOLDER=BASE/'whole_hand_preflight_v2'
KEY='whole_hand_native_result_20261110'


def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def remove_block(text,key):
    start='<!-- '+key+' -->';end='<!-- '+key+'_end -->'
    if start in text:
        begin=text.index(start);finish=text.index(end,begin)+len(end)
        text=text[:begin]+text[finish:].lstrip('\n')
    return text


def markdown(path,body,after=None):
    text=path.read_text(encoding='utf-8')
    for key in ('whole_hand_preflight_20261110',KEY):text=remove_block(text,key)
    block='<!-- '+KEY+' -->\n'+body.rstrip()+'\n<!-- '+KEY+'_end -->\n\n'
    if after:
        position=text.index(after)+len(after);text=text[:position]+'\n\n'+block+text[position:].lstrip('\n')
    else:text=block+text
    path.write_text(text,encoding='utf-8')


def banner(path,relative):
    text=path.read_text(encoding='utf-8')
    for key in ('whole_hand_preflight_20261110',KEY):text=remove_block(text,key)
    body=re.search(r'<body\b[^>]*>',text,re.I)
    content=re.search(r'<(?:h[1-6]|main|section)\b',text,re.I) if body is None else None
    position=body.end() if body else content.start()
    joined_path=BASE/'RF_clear_work_grip_joined_suffix/review.json'
    latest=('검지·약지 공간 확보와 파지 유지392목표를 모델에서 연결했습니다. 출발은 별도로 배치한 약지 파지입니다. 초기 배치의 회전과 다음 회전 연결은 남아 있습니다. '
            if joined_path.exists() and read(joined_path)['conditional_clear_work_grip_suffix_passed'] else
            '첫 회전4118목표에서 모델 손–큐브 접촉 소실을 확인했습니다. 지지가 남은 중간 상태의 보정 결과를 기록했습니다. ')
    panel=('<!-- '+KEY+' --><aside style="margin:16px;padding:16px;border:2px solid #397b75;background:#edf8f5">'
        '<strong>2026-11-10 모델 검토:</strong> '+latest+
        '<a href="'+html.escape(relative,quote=True)+'">최신 실행 결과</a></aside><!-- '+KEY+'_end -->')
    path.write_text(text[:position]+panel+text[position:],encoding='utf-8')


def automatic_studies():
    paths={
        'SAC_gain_sensitivity':BASE/'sac_gain_sensitivity/review.json',
        'palm_contact_persistence':BASE/'palm_contact_persistence/review.json',
        'palm_contact_convergence':BASE/'palm_contact_convergence/review.json',
        'RF_checkpoint_search':BASE/'supported_checkpoint_ring/review.json',
        'RF_global_reach':BASE/'ring_global_reach/review.json',
        'RF_transient_global_reach':BASE/'ring_global_reach_transient/review.json',
        'RF_endpoint_current_model_holds':BASE/'RF_endpoint_CCD50_load_screen/review.json',
        'RF_reference_arrivals':BASE/'RF_reference_arrivals/review.json',
        'RF_upper_blockers':BASE/'RF_upper_blockers/review.json',
        'RF_contact_patch_refinement':BASE/'RF_work_grip_refinement/review.json',
        'RF_index_clearance_stage':BASE/'RF_index_clearance_stage/review.json',
        'RF_index_clearance_margin2':BASE/'RF_index_clearance_margin2/review.json',
        'RF_clear_global_after_index':BASE/'RF_clear_global_after_index/review.json',
        'RF_clearance_free_arrivals':BASE/'RF_clearance_free_arrivals/review.json',
        'RF_clear_work_grip_joined_suffix':BASE/'RF_clear_work_grip_joined_suffix/review.json',
        'clear_work_grip_reference_arrivals':BASE/'clear_work_grip_reference_arrivals/review.json',
        'clear_work_grip_load_hypotheses':BASE/'clear_work_grip_load_hypotheses/review.json',
        'geometry_analytic_checks':OUT/'convex_pair_gap_verification.json'}
    studies={};paragraphs=[]
    for name,path in paths.items():
        if not path.exists():continue
        data=read(path);item=dict(path=path.relative_to(OUT).as_posix(),sha256=sha(path))
        if name=='SAC_gain_sensitivity':
            item.update(rows=[{key:row[key] for key in ('gain_rad','steps','passed','stop','best_sampled_error_deg')} for row in data['rows']],
                actor_weights_unchanged=data['actor_weights_unchanged'],new_SAC_training_steps=data['new_SAC_training_steps'],
                baseline_reproduced=data['original_gain_with_projection_goals_equal_previous'])
            paragraphs.append('동일 SAC 가중치의 보정 폭0.01/0.03/0.05rad를 비교했다. '+
                ' / '.join(f"{row['gain_rad']:.2f}rad: {row['steps']}목표, 종료 {','.join(row['stop'] or [])}" for row in data['rows'])+
                '. 세 조건 모두 첫 회전·유지 미달이며 새 SAC 학습은0스텝이다. 보정 폭 조절만으로 해결됐다고 판단하지 않는다.')
        elif name=='palm_contact_persistence':
            source=data['rows'][0]
            item.update(rows=[{key:row[key] for key in ('source','simulated_substeps','stop','original_strict_hold_passed',
                'maximum_cube_angle_drift_deg','maximum_cube_center_drift_mm','supported_fraction')} for row in data['rows']])
            paragraphs.append(f"기존1700 상태는 원래 기준으로4.05초 유지했고 최대 방향 변화{source['maximum_cube_angle_drift_deg']:.3f}도, 위치 변화{source['maximum_cube_center_drift_mm']:.3f}mm, 전체 손 지지 표본100%였다. 목표90도에 도달한 파지는 아니다. 후속121/165 끝 자세의 관찰은 다른 손 접촉 기준에도 걸려47/46ms에 중단됐으므로4초 유지 성공으로 세지 않는다.")
        elif name=='palm_contact_convergence':
            groups={}
            for row in data['rows']:
                groups.setdefault(row['source'],[]).append({p['palm'] for p in row['pairs'] if any(v<0 for v in p['native_contact_distances_mm'])})
            unchanged=all(all(group==values[0] for group in values) for values in groups.values())
            item.update(states=data['states'],comparisons=len(data['rows']),
                convex_native_contact_contradictions=len(data['positive_convex_gap_with_negative_native_contact']),
                negative_contact_pair_sets_equal_across_iteration_caps=unchanged)
            paragraphs.append('중지–손바닥 접촉을 독립 볼록 형상과CCD50/100/200/500회로9자세·36비교했다. 양의 형상 간격인데 음의 native 접촉이 생기는 모순은 이 비교에서0개였다. 이 작은 모델 접촉을 실물 위험 충돌로 일반화하지 않는다.')
        elif name=='RF_checkpoint_search':
            closest=min(data['fits'],key=lambda row:row['point_error_mm'])
            item.update(fits=len(data['fits']),static_candidates=data['static_candidates'],dynamic_trials=len(data['dynamic_trials']),
                        closest_target_point_error_mm=closest['point_error_mm'],temporary_RF_support_passed=data['temporary_RF_support_passed'])
            paragraphs.append(f"다른 손가락과 큐브 위치를 유지한 약지 접촉 탐색144개 중 정적 후보는{data['static_candidates']}개였다. 목표점 선택과 두 초기값을 사용한 유한 탐색이므로 약지의 일반적 도달 불가를 증명하지 않는다.")
        elif name in ('RF_global_reach','RF_transient_global_reach'):
            item.update(rows=[{key:row[key] for key in ('profile','geom','selected_signed_convex_gap_mm',
                'static_contact_geometry_candidate','original_range_violation_rad')} for row in data['rows']],
                actual_motion_evaluations=data['actual_motion_evaluations'],
                physical_and_model_limits_unchanged=data['model_and_physical_joint_limits_not_modified'])
            counts={profile:sum(row['static_contact_geometry_candidate'] for row in data['rows'] if row['profile']==profile)
                    for profile in ('original_model_bound','recorded_ID14_geometry_hypothesis')}
            item['geometric_candidates_by_profile']=counts
            if name=='RF_global_reach':
                paragraphs.append(f"하부 지지의 전역 탐색 형상 후보는 기존 모델 상한에서{counts['original_model_bound']}개, 기록 ID14 각도 가설에서{counts['recorded_ID14_geometry_hypothesis']}개다. 가설 비교는 정적 형상만 계산했고 실물·원본 모델 제한은 바꾸지 않았다. 도달 경로나 지지력 검증이 아니다.")
            else:
                paragraphs.append(f"회전 중 윗부분 접촉도 허용하는 임시 지지의 전역 탐색 형상 후보는 기존 모델 범위에서{counts['original_model_bound']}개다. 임시 지지와 최종 윗단 공간 확보를 분리한 정적 형상 비교이며 도달·유지 성공이 아니다.")
        elif name=='RF_endpoint_current_model_holds':
            item.update(rows=[{key:row[key] for key in ('source','steps_1ms','stop','initialized_RF_hold_passed',
                'final_global90_error_deg','upper_strict_min_gap_mm','translation_from_checkpoint1700_norm_mm',
                'initialized_work_grip_fully_certified')} for row in data['rows']])
            held=sum(row['initialized_RF_hold_passed'] for row in data['rows'])
            item['negative_separation_certificate_value_is_not_penetration_depth']=True
            paragraphs.append(f"이전 약지 지지 끝 자세4개를 현재CCD50 모델에서 재검증해{held}개가 초기화 후4.05초 지지를 유지했다. 절대90도 목표와의 오차는 약0.62~0.95도였지만 마지막 자세의 윗단 분리 인증 하한은 음수여서 공간 확보를 입증하지 못했다. 이 음수 값을 실제 침범 깊이로 해석하지 않는다. 이 자세들은 현재 중간 상태에서 큐브 중심이 약23mm 떨어져 있고 다른 손가락 자세도 달라, 단순 약지 굽힘과 전체 파지 이동을 구분해야 한다. 도달 경로 성공이나 윗단 공간 인증 완료가 아니다.")
        elif name=='RF_reference_arrivals':
            item.update(rows=[{key:row[key] for key in ('order','steps','stop','final_global90_error_deg',
                'first_rotation_and_RF_hold_from_checkpoint_passed')} for row in data['rows']],
                cube_free_through_every_arrival=data['cube_free_through_every_arrival'])
            passed=sum(row['first_rotation_and_RF_hold_from_checkpoint_passed'] for row in data['rows'])
            paragraphs.append(f"검증한 약지 지지 자세로 가는 네 이동 순서를 자유 동역학으로 비교했고{passed}개가 도달·유지 조건을 통과했다. 이번 네 경로는 손 접촉 또는 추종 기준에서 중단됐다. 끝 자세에서 유지할 수 있다는 결과와 그 자세까지 안전하게 이동했다는 결과를 구분한다.")
        elif name=='RF_upper_blockers':
            item.update(confirmed_geoms=[row['geom'] for row in data['rows'] if row['witness_within_required_margin']],
                source_sha256=data['source_sha256'],geometry_witnesses=data['rows'])
            paragraphs.append('별도로 배치한 약지 지지 파지에서 검지 끝·약지 끝과 약지 끝마디 형상이 윗단 회전 영역 안에 놓이는 모델 점을 확인했다. 음의 분리 인증 하한을 침범 깊이로 바꿔 쓰지 않았다.')
        elif name=='RF_contact_patch_refinement':
            item.update(fits=len(data['fits']),static_candidates=data['static_candidates'],dynamic_trials=len(data['dynamic_trials']))
            paragraphs.append('현재 약지 접촉점의 같은 부분을 아래로 옮기는18개 정적 탐색은 후보0개였다. 이후 접촉점을 고정하지 않는 전역 탐색과 검지·약지 단계 분리로 전환했다.')
        elif name in ('RF_index_clearance_stage','RF_index_clearance_margin2'):
            item.update(optimization_margin_mm=data.get('optimization_margin_mm',.65),
                trials=[{key:row.get(key) for key in ('path','steps','stop','hold_samples','whole_hand_support_fraction','RF_loaded_fraction','IF_clear_samples',
                    'IF_certificate_minimum_lower_mm','hold_angle_drift_deg','hold_center_drift_mm','initialized_IF_clear_whole_hand_hold_passed')} for row in data['dynamic_trials']])
            if name=='RF_index_clearance_margin2':
                row=data['dynamic_trials'][0]
                paragraphs.append(f"검지 뿌리와 다른 손가락 지령을 유지하고 검지 끝만 옮겼다. 원래 구동·충돌 조건에서{row['steps']}목표를 실행했고 마지막4050상태 모두 손 전체 지지를 유지했다. 검지 윗단 분리 인증 최소 하한은{row['IF_certificate_minimum_lower_mm']:.3f}mm다. 약지의0.2N 초과 비율은 별도 진단이며 당시 약지 쪽 윗단 공간은 여전히 미달이다.")
        elif name=='RF_clear_global_after_index':
            item.update(proposals=[{key:row[key] for key in ('geom','signed_axis_gap_mm','RF_screen_mm','bad_upper_geoms','static_clear_work_grip_candidate')} for row in data['proposals']])
            paragraphs.append('검지 공간 확보 뒤 약지의 끝·끝마디·중간마디를 대상으로 원래 범위 안에서 전역 탐색했다. 공간을 비우는 자세는 찾았지만 정적 약지 접촉 후보는0개였다. 이 자세에서 손 전체 지지가 유지되는지는 별도로 자유 동역학 검사했다.')
        elif name=='RF_clearance_free_arrivals':
            item.update(trials=[{key:row[key] for key in ('path','steps','stop','hold_samples','whole_hand_support_fraction','RF_contact_fraction',
                'initialized_clear_work_grip_suffix_passed','certified_window_samples','upper_certificate_minimum_lower_mm')} for row in data['trials']])
            paragraphs.append('약지 공간 확보 제안3개로 실제 자유 동역학 이동·유지를 비교했다. 하나가 자세·손 전체 지지·최종4050상태 공간 인증을 통과했다. 통과 자세의 약지 접촉력은0N이며 검지 뿌리·검지 마디·엄지가 지지했다. 약지 세 면 고정의 성공으로 세지 않는다.')
        elif name=='RF_clear_work_grip_joined_suffix':
            item.update({key:data[key] for key in ('steps','stage_goals','stop','seams','actual_state_resets_after_start','final_hold_samples','certified_samples',
                'upper_certificate_minimum_lower_mm','whole_hand_support_fraction','final_global90_error_deg','common_station_error_mm',
                'hold_angle_drift_deg','hold_center_drift_mm','conditional_clear_work_grip_suffix_passed')})
            paragraphs.append(f"검지90목표와 약지302목표를 실제 상태 초기화 없이392목표로 재생했고 두 구간 끝 상태가 별도 실행과 정확히 일치했다. 마지막4050상태 전체 손 지지100%, 윗단 인증 하한{data['upper_certificate_minimum_lower_mm']:.3f}mm, 목표 자세 오차{data['final_global90_error_deg']:.3f}도, 공통 위치 오차{data['common_station_error_mm']:.3f}mm다. 출발 약지 파지는 별도 초기화한 것이므로 초기 배치의 첫 회전·다음 회전이나 실물 성공은 아니다.")
        elif name=='clear_work_grip_reference_arrivals':
            item.update(rows=[{key:row[key] for key in ('order','steps','stop','final_global90_error_deg','first_rotation_and_work_grip_from_checkpoint_passed')} for row in data['rows']])
            paragraphs.append('실제1700 회전 중간 상태에서 새 작업 파지로 가는 네 이동 순서를 자유 동역학으로 검사했다. 통과 수는'+str(sum(row['first_rotation_and_work_grip_from_checkpoint_passed'] for row in data['rows']))+'개다. 작업 파지에서 지탱하는 것과 그 파지까지 들어오는 경로를 분리해 기록했다.')
        elif name=='clear_work_grip_load_hypotheses':
            item.update(cases=[{key:row[key] for key in ('case','assumed_sliding_friction','assumed_constant_moment_about_up_Nm',
                'samples','stop','conditional_hold_passed','angle_drift_deg','final_bad_upper_geoms')} for row in data['cases']],
                applied_moments_are_not_measured_left_hand_loads=True)
            paragraphs.append('새 작업 파지에서 가정한 마찰·회전 하중11조건을 검사해7조건이 유지·공간 인증을 통과했다. 무부하 마찰0.25/0.30/0.35와 양방향0.0005/0.001N·m는 통과했다. −0.003N·m는667ms에 외부 받침 접촉으로 중단됐고 +0.003N·m는 자세가 약0.97도 변한 채 약지 끝마디의 윗단 공간이 미달했다. ±0.01N·m도 미달이다. 시험한 이산 조건만의 결과이며 실제 왼손 하중이나450mA 환산을 측정한 값이 아니다.')
        else:item.update(tests_run=data['tests_run'],status=data['status'])
        studies[name]=item
    return dict(user_requested_automatic_approval=True,studies=studies,new_SAC_training_steps=0,
                hardware_execution_allowed=False), '\n\n'.join(paragraphs)


def main():
    execution=read(FOLDER/'native_execution.json')
    if execution['status']!='native_preflight_reports_created':raise RuntimeError('Native queue has not completed')
    raw_path=BASE/'recorded_ring_reassessment/geometry_audit.json';raw=read(raw_path)
    state_path=BASE/'whole_hand_saved_state_audit/review.json';states=read(state_path)
    trial_path=OUT/execution['tasks'][-1]['output'];trial=read(trial_path)
    old_folder=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_active_ccd50'
    a=np.load(trial_path.parent/'research_trace.npz',allow_pickle=False)['goals']
    b=np.load(old_folder/'research_trace.npz',allow_pickle=False)['goals']
    identical=np.array_equal(a,b)
    if not identical:raise RuntimeError('Accounting-only comparison changed control commands')
    end=np.load(trial_path.parent/'end_state.npz',allow_pickle=False);old=np.load(old_folder/'end_state.npz',allow_pickle=False)
    differences={name:float(np.max(abs(end[name]-old[name]))) if end[name].size else 0.
                 for name in ('qpos','qvel','ctrl','act','qacc_warmstart')}
    supported=[row for row in trial['trace'] if row['whole_hand_support']['supported_sample']]
    best=min(trial['trace'],key=lambda row:row['error_deg'])
    feedback=[]
    for folder in sorted(OUT.glob('next_cycle/whole_hand_recovery_cp*')):
        if (folder/'review.json').exists():
            r=read(folder/'review.json')
            feedback.append(dict(path=(folder/'review.json').relative_to(OUT).as_posix(),sha256=sha(folder/'review.json'),
                completed=True,steps=r['steps'],stop=r['stop'],start_error_deg=r['start']['error_deg'],
                best_error_deg=r['best_orientation_error_deg'],final_error_deg=r['final']['error_deg'],
                local_target_screen_held=r['local_target_screen_held'],full_cycle_or_physical_success=False,
                goal_reference_state=r.get('goal_reference_state'),
                goal_reference_sha256=r.get('goal_reference_sha256'),
                joint_reference_weight=r.get('joint_reference_weight',0.),
                final_center_target_error_mm=r['final'].get('center_target_error_mm',r['final']['center_drift_from_checkpoint_mm']),
                final_RF_normal_N=sum(value['normal_N'] for name,value in r['final']['whole_hand_support']['by_geom'].items() if name.startswith('rf_'))))
        else:
            progress=read(folder/'progress.json') if (folder/'progress.json').exists() else None
            feedback.append(dict(path=folder.relative_to(OUT).as_posix(),completed=False,progress=progress,
                                 full_cycle_or_physical_success=False))
    exact=[dict(source=row['source'],nearest_model_RF_gap_mm=row['closest_rf'][0]['signed_SAT_gap_mm'],
                gap_is_separating_axis_lower_bound_not_exact_distance=True,
                original_guard_self_mm=row['original_guard']['self_mm'],
                original_guard_range_rad=row['original_guard']['range_rad'])
           for row in raw['results'] if row['mode']=='exact_recorded_reference']
    auto_summary,auto_text=automatic_studies()
    auto_brief=('기존 SAC 보정 폭3조건과 중지–손바닥 형상36비교, 지지 상태의 약지144탐색 및 별도 전역 탐색을 수행했다. '
                '새 SAC 학습은0스텝이며 자세·구동·충돌 기준을 완화해 성공으로 바꾸지 않았다. '
                '회전 중 임시 접촉과 최종 윗단 공간 조건을 구분하며 상세는 최신 실행 보고서를 따른다.') if auto_text else ''
    joined_path=BASE/'RF_clear_work_grip_joined_suffix/review.json'
    joined=read(joined_path) if joined_path.exists() else None
    milestone=('**최신 진전:** 별도 약지 파지에서 검지 공간 확보→약지 이동→파지 유지392목표를 중간 상태 초기화 없이 연결했다. '
        f"마지막4050상태 전체 손 지지100%, 윗단 공간 인증 하한{joined['upper_certificate_minimum_lower_mm']:.3f}mm, 목표 자세 오차{joined['final_global90_error_deg']:.3f}도다. "
        '마지막 약지 접촉력은0N으로, 검지 뿌리 등을 이용한 지지다. 초기 배치에서 이 파지까지 들어가는 회전과 다음 회전 연결은 남아 있다. '
        '모델 비교 그림·영상은 `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/RF_clear_work_grip_evidence/index.html`에 있다.') if joined else ''
    diagnosis_path=OUT/'next_cycle/whole_hand_recovery_cp1700_w0.15/stop_diagnosis/review.json'
    diagnosis=read(diagnosis_path) if diagnosis_path.exists() else None
    diagnosis_summary=None
    if diagnosis:
        first=next(row for row in diagnosis['rows'] if row['stop'])
        diagnosis_summary=dict(path=diagnosis_path.relative_to(OUT).as_posix(),sha256=sha(diagnosis_path),
            proposals=diagnosis['proposals'],admissible_proposals=len(diagnosis['safe_proposals']),
            rejection_counts=diagnosis['rejection_counts'],
            first_rejected_self_pairs_mm=first.get('last_guard',{}).get('self_pairs_mm',{}),
            not_a_physical_collision_risk_measurement=True,
            tested_candidate_failure_is_not_a_general_infeasibility_proof=True)
    report=dict(status='native_whole_hand_preflight_executed',date='2026-11-10',
        user_changed_approval_mode_and_native_execution_succeeded=True,
        account_usage_reset_or_credit_purchase_not_inferred=True,
        native_queue_tasks_completed=3,recorded_ring_geometry_queries=len(raw['results']),
        exact_recorded_ring_model_comparisons=exact,recorded_real_cube_pose_unregistered=True,
        unchanged_recorded_ring_source_sha256=raw['recorded_source_sha256'],
        full_hand_saved_state_queries=len(states['rows']),all_force_frame_checks_passed=states['all_force_frame_checks_passed'],
        completed_first_rotation_trial=dict(path=trial_path.relative_to(OUT).as_posix(),sha256=sha(trial_path),
            steps=trial['steps'],passed=trial['first_rotation_and_hold_passed'],stop=trial['stop'],
            final=trial['final'],terminal_hand_support=trial['terminal_hand_support'],
            actual_terminal_no_hand_cube_contacts=not trial['terminal_hand_support']['by_geom'],
            terminal_native_guard=trial['terminal_native_guard']),
        same_parent_goals_reproduced=identical,end_state_max_abs_difference_from_parent=differences,
        sampled_trace_supported_states=len(supported),sampled_trace_states=len(trial['trace']),
        sampled_trace_is_not_full_1ms_history=True,
        best_sampled_orientation=dict(step=best['step'],error_deg=best['error_deg'],
                                     tilt_deg=best['tilt_deg'],common_station_error_mm=best['center_error_mm']),
        model_feedback_recovery_probes=feedback,first_rotation_and_hold_certified=False,
        local_feedback_stop_diagnosis=diagnosis_summary,
        automatic_continuation=auto_summary,
        next_rotation_passed=False,whole_cycle_ready_for_hardware=False,
        devices_opened=0,hardware_execution_allowed=False,
        source_reports_sha256={path.relative_to(OUT).as_posix():sha(path) for path in (raw_path,state_path,FOLDER/'native_execution.json')})
    (FOLDER/'native_summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    feedback_text='국소 보정 계산 중이며 최종 보고서는 아직 없다.'
    completed=[row for row in feedback if row['completed']]
    if completed:
        feedback_text=' / '.join(f"{row['steps']}목표, 시작 오차{row['start_error_deg']:.3f}도, 최소{row['best_error_deg']:.3f}도, 종료{row['stop']}" for row in completed)
    diagnosis_text=('첫 보정의 정지 상태에서98개 예측을 비교했고97개가 중지 끝–손바닥 모델 접촉 기준에 걸렸다. '
        '유지 지령 예측의 최초 겹침은0.000223mm였다. 실물 위험 충돌의 측정값이 아니다. '
        '여러 관절을 함께 움직이는 후보1개는 원래 기준을 통과했다. 이를 포함한 보정은165목표까지 진행했고 '
        '최소 방향 오차23.557도였으며 최종 작업 목표는 미달이다. 작은 모델 겹침 때문에 멈춘 국소 보정과 '
        '접촉을 잃고 내려간 기존 첫 회전의 결과를 구분한다.') if diagnosis else ''
    if diagnosis and not any('coupled/' in row['path'] for row in completed):
        diagnosis_text='98개 예측 중 동시 관절 후보1개가 원래 기준을 통과했다. 후속 보정은 아직 계산 중이다.'
    text=f'''# 전체 손 지지: 실제 모델 실행 결과 — 2026-11-10

{milestone}

사용자가 승인 방식을 변경한 후 WSL MuJoCo 실행이 승인됐고 준비한 검사3개를 실제로 실행했다. 앞선 자동 승인 검토 사용량 오류가 계정 충전이나 한도 초기화로 해결됐다고 추정하지 않는다. 현재 실행은 사용자 승인 경로로 진행됐다.

- 기록한 약지 네 값 그대로와 잘린 값을 모델 자세3곳에서 비교했다. 총9개 정적 형상 조회에서 손 자기 충돌은 없었다. 정확한 네 값을 넣었을 때 RF 형상과 큐브의 분리 간격에 대한 SAT 하한은 약35.4/36.8/52.8mm로 양수여서 이 세 배치에서는 접촉하지 않았다. 정확한 최근접 거리나 실물 간격을 측정한 값은 아니다. 당시 실물 큐브 위치를 등록한 기록이 없으므로 기록 자세가 실물에서 지지할 수 없다는 결론이 아니다. ID14가 모델 상한을 넘는 사실은 물리적 도달 불가의 증거로 쓰지 않는다.
- 저장 상태6개의 전체 손·외부 접촉을 계산했고 힘의 좌표계/부호를 일반화 구속력과 대조하는 검사를 전부 통과했다.
- 손바닥·뿌리·약지를 포함한 지지 평가로 첫 회전부터 다시 실행했다. 기존 제어 목표4118개와 끝 상태가 정확히 일치했고 같은80mm 이동 기준에서 중단됐다. 중단 시점에는 손–큐브 접촉이 없고 손의 위쪽 지지력은0N이었다. 이번에는 단순 이동 거리 외에 모델 접촉 소실도 확인했다. 실물 실행이나 실물 낙하가 아니다.
- 제어100주기 간격의 저장 표본43개 중42개에서는 전체 손 지지 표본 조건을 만족했다. 연속1ms 전체 이력의 지지 비율은 아니다. 가장 작은 표본 방향 오차는1701목표에서28.294도였고 공통 위치 오차는21.952mm였다. 동작 전체가90도 목표를 충족했다고 판단하지 않는다.

지지가 유지되던1700목표 상태에서 기준 궤적의 후속 목표를 고정해 재생하는 대신 실제 모델 피드백으로90도 방향을 향하는 국소 보정을 검사했다. 이 보정은 모델 기반 교사 연구이며 SAC 학습 또는 증류 완료가 아니다. 현재 큐브 중심을 유지하는 진단이므로 최종 공통 파지 위치·엄격한 윗단 인증·전체 연결 성공과 구분한다.

보정 상태: {feedback_text}

{diagnosis_text}

## 자동 승인으로 이어 한 계산

{auto_text}

전체 실물 준비 완료가 아니다. 현재 사용자의 로봇 재연결·촬영·반복 시연은 필요하지 않다. 원래170개 결합 파일과450mA/PID/2틱/75ms 조건을 유지한다.

[실행 요약](native_summary.json) · [약지 비교](../recorded_ring_reassessment/geometry_audit.json) · [접촉 검산](../whole_hand_saved_state_audit/review.json) · [실행 전 준비 기록](STATUS.md)
'''
    (FOLDER/'NATIVE_STATUS.md').write_text(text,encoding='utf-8')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>전체 손 지지 모델 실행 결과</title><style>body{font:17px/1.75 system-ui,sans-serif;max-width:1020px;margin:32px auto;padding:0 20px;color:#183c34;background:#f5f9f7}section{padding:20px;background:white;border:1px solid #cadfd4;margin:20px 0}a{color:#176f63}</style><body><h1>전체 손 지지: 모델 실행 결과</h1><p><strong>승인 경로 변경 후 모델 검사3개 실행 완료 / 전체 실물 준비 미완료</strong></p><section><h2>확인한 내용</h2><ul><li>기록 약지 네 값을 그대로 넣는 형상 비교9개 완료. 비교한 세 큐브 배치에서는 약지 접촉이 없었습니다. 당시 실물 큐브 위치가 등록된 것은 아닙니다.</li><li>저장 모델 상태6개의 접촉력·힘 방향 검산 통과.</li><li>첫 회전4118목표에서 중단. 손 전체의 큐브 접촉과 지지력은0이었습니다. 실제 로봇 실행은 아닙니다.</li></ul></section><section><h2>이어 한 보정</h2><p>손바닥과 손가락 지지가 유지되던 중간 상태에서 실제 모델 피드백으로 목표 방향에 접근하는지 확인했습니다.</p><p>'''+html.escape(feedback_text)+'''</p><p>'''+html.escape(diagnosis_text)+'''</p><p>국소 보정·SAC 학습·전체 회전 연결을 구분합니다.</p></section><p><a href="NATIVE_STATUS.md">상세 결과</a> · <a href="native_summary.json">실행 요약</a> · <a href="STATUS.md">실행 전 준비 기록</a></p></body></html>'''
    if auto_text:
        panel='<section><h2>자동 승인 후속 계산</h2>'+''.join('<p>'+html.escape(p)+'</p>' for p in auto_text.split('\n\n'))+'</section>'
        page=page.replace('</body>',panel+'</body>')
    if joined:
        panel='<section><h2>최신: 윗단 공간을 비운 작업 파지</h2><p>'+html.escape(milestone.replace('**',''))+'</p><p><a href="../RF_clear_work_grip_evidence/index.html">전후 비교 그림·모델 영상</a> · <a href="../RF_clear_work_grip_joined_suffix/review.json">392목표 연결 근거</a></p></section>'
        page=page.replace('<section><h2>확인한 내용',panel+'<section><h2>확인한 내용',1)
    loads_path=BASE/'clear_work_grip_load_hypotheses/review.json'
    if loads_path.exists():
        loads=read(loads_path)
        load_rows=''.join('<tr><td>'+html.escape(row['case'])+'</td><td>'+('통과' if row['conditional_hold_passed'] else '미달')+'</td><td>'+f"{row['angle_drift_deg']:.3f}도"+'</td><td>'+html.escape(', '.join(row['stop']+row['final_bad_upper_geoms']) or '없음')+'</td></tr>' for row in loads['cases'])
        load_panel='<section><h2>가정한 마찰·회전 하중의 추가 검사</h2><p>아래 결과는 별도 초기화한 작업 파지에서 지령을 유지한 모델 검사다. 실제 왼손 하중 측정이나 실물 허용치가 아니다. 두 방향의0.001N·m까지 시험 조건은 통과했지만0.003N·m부터 자세·공간 또는 지지가 미달했다.</p><table><tr><th>조건</th><th>유지·공간</th><th>최대 방향 변화</th><th>중단/마지막 공간 미달</th></tr>'+load_rows+'</table><p><a href="../clear_work_grip_load_hypotheses/review.json">모든 수치와 가정</a></p></section>'
        page=page.replace('</body>',load_panel+'</body>')
        visual_path=BASE/'RF_clear_work_grip_evidence/index.html'
        if visual_path.exists():
            visual=remove_block(visual_path.read_text(encoding='utf-8'),'RF_clear_loads')
            block='<!-- RF_clear_loads -->'+load_panel+'<!-- RF_clear_loads_end -->'
            visual_path.write_text(visual.replace('<section><h2>남은 연결',block+'<section><h2>남은 연결',1),encoding='utf-8')
    (FOLDER/'index.html').write_text(page,encoding='utf-8')
    latest=f'''{milestone}

2026-11-10: **WSL MuJoCo 검사를 실제 실행했다.** 이전의 “승인 검토 사용량 오류로 native 미실행”은 앞선 준비 기록이다. 현재 사용자 요청에 따라 자동 승인을 사용하며 승인 검토의 사용량·계정 충전이나 한도 초기화를 추정하지 않는다. 기록 약지 자세9개 정적 비교에서 자기 충돌은 없었지만, 비교한 세 모델 큐브 배치에서 약지는 접촉하지 않았다. 당시 실물 큐브 위치는 등록되지 않았으므로 실물 자세 자체의 지지 불가로 판단하지 않는다.

저장 상태6개의 힘 검산을 통과했다. 전체 손 지지를 포함한 초기 배치부터의 첫 회전 재시험은4118목표에서80mm 이동으로 중단됐으며, 이번에는 실제 모델 손–큐브 접촉 소실과 지지력0N을 확인했다. 기존 목표 및 끝 상태와 차이0이다. 손바닥 지지가 있는 중간 상태1700에서 기준 궤적을 계속 따라가는 대신 모델 피드백 보정을 검사했다. {feedback_text}

{diagnosis_text}

자동 승인 후속: {auto_brief}

추가 유지·공간 검사: 가정한 마찰0.25/0.30/0.35와 양방향0.001N·m 하중 조건은 새 작업 파지에서 통과했다. 양방향0.003N·m와0.01N·m는 미달이다. 실제 왼손 하중·실물 허용치의 측정값이 아니다. 지금은 무부하 작업 파지 후보를 확보한 것이며 고정력 검증까지 완료한 것이 아니다.

보정 연구는 SAC 추가 학습·증류·공통 최종 파지·다음 회전 연결의 완료가 아니다. 전체 실물 준비는 미완료이며 지금 사용자 재연결/촬영/반복 시연은 요구하지 않는다. 최신 근거: `right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/whole_hand_preflight_v2/NATIVE_STATUS.md`, `native_summary.json`, `index.html`.'''
    markdown(ROOT/'cloud_handoff_20261106/CLOUD_START_PROMPT.md',latest,after='## 지금 먼저 할 일')
    markdown(ROOT/'professor_progress_20261015.md',latest)
    markdown(BASE/'STATUS.md','모델 검사3개를 실제 실행했다. [최신 실행 결과](whole_hand_preflight_v2/index.html)를 먼저 본다. 이전 승인 사용량 오류로 미실행인 내용은 준비 당시 기록이다.',after='# 약지 하중 인계 후속: 2026-11-10')
    banner(BASE/'index.html','whole_hand_preflight_v2/index.html')
    banner(OUT/'review/index.html','../next_cycle/handoff_refinement_20261110/whole_hand_preflight_v2/index.html')
    banner(ROOT/'right_hand_rotation_20261109/policy_audit/explanation/index.html','../../prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/whole_hand_preflight_v2/index.html')
    readiness_path=OUT/'next_cycle/readiness.json';readiness=read(readiness_path)
    readiness['whole_hand_support_preparation'].update(native_evaluation_pending=False,exact_recorded_ring_comparison_pending=False)
    readiness['whole_hand_native_results']=dict(path=(FOLDER/'native_summary.json').relative_to(OUT).as_posix(),
        sha256=sha(FOLDER/'native_summary.json'),native_tasks_completed=3,first_rotation_and_hold_passed=False,
        whole_cycle_ready_for_hardware=False,hardware_execution_allowed=False)
    readiness_path.write_text(json.dumps(readiness,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(native_tasks_completed=3,first_rotation_passed=False,
                         feedback_completed=len(completed),hardware_execution_allowed=False)),flush=True)


if __name__=='__main__':main()
