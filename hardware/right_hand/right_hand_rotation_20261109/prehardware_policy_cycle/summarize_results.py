"""Compact durable overview of completed policy trials, including failures."""
from pathlib import Path
import json
OUT=Path(__file__).resolve().parent


def save_json(path,result):
    path.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')


def main():
    rows=[]
    for path in sorted(OUT.glob('sac_*/export_*/evaluation_*/review.json')):
        r=json.loads(path.read_text());e=r['endpoint']
        export_manifest=json.loads((path.parent.parent/'manifest.json').read_text())
        rows.append({'path':path.relative_to(OUT).as_posix(),'checkpoint':r['checkpoint'],
                     'is_original_actor_before_new_training':export_manifest.get('is_original_actor_before_new_training',False),
                     'case':r.get('case','nominal'),'passed':r['rotation_and_hold_passed'],'reasons':r['reasons'],
                     'yaw':e['yaw_deg'],'target_error':e['target90_error_deg'],
                     'lower_fraction':r['settled_lower_contact_fraction'],
                     'upper_gap':r['settled_upper_gap_sampled_75ms_mm'],
                     'closest':e['closest_upper_geoms'][:2],
                     'station_error':r.get('common_station_error_mm'),
                     'certified_every_1ms':not r['separation_certificate_1ms_not_yet_run']})
    training=[]
    for path in sorted(OUT.glob('sac_*/result.json')):
        r=json.loads(path.read_text())
        training.append({'path':path.relative_to(OUT).as_posix(),'status':r['status'],
                         'steps':r.get('steps',r.get('timesteps')),
                         'elapsed_seconds':r.get('elapsed_seconds'),
                         'trainer_reported_environment_steps':json.loads((path.parent/'progress.json').read_text()).get('steps') if (path.parent/'progress.json').exists() else None,
                         'numerical_CCD_iterations':r.get('numerical_CCD_iterations',10)})
    small_turn=[]
    for path in sorted(OUT.glob('sac_next_*/evaluation_*/review.json')):
        r=json.loads(path.read_text());small_turn.append({'path':path.relative_to(OUT).as_posix(),
            'increment_deg':r['increment_deg'],'passed':r['success'],
            'initial_error_deg':r['initial_goal_error_deg'],'final_error_deg':r['final_goal_error_deg']})
    teacher=[]
    for path in sorted(OUT.glob('next_model_feedback_*/review.json')):
        r=json.loads(path.read_text());teacher.append({'path':path.relative_to(OUT).as_posix(),
            'increment_deg':r['curriculum_turn_deg'],'passed':r['success'],
            'final_error_deg':r['final']['error_deg'] if r['final'] else None,
            'stop':r['stop'],'is_SAC':False})
    connected_path=OUT/'connected_sac_and_teacher/review.json'
    connected=json.loads(connected_path.read_text()) if connected_path.exists() else None
    if connected:connected.pop('trace',None)
    learned_connections=[]
    for path in sorted(OUT.glob('connected_sac_student_hold_*/review.json')):
        r=json.loads(path.read_text());r.pop('trace',None)
        learned_connections.append(dict(path=path.relative_to(OUT).as_posix(),**r))
    packaged=[]
    for path in sorted(OUT.glob('next_cycle/staged_model_*/review.json')):
        r=json.loads(path.read_text());r.pop('trace',None)
        packaged.append(dict(path=path.relative_to(OUT).as_posix(),**r))
    corrections=[]
    for path in sorted(OUT.glob('next_cycle/recovery_bc_*/training.json')):
        r=json.loads(path.read_text());corrections.append(dict(path=path.relative_to(OUT).as_posix(),training=r))
    result={'rows':rows,'training':training,'new_training_steps':sum(r['steps'] or 0 for r in training),
            'small_turn_SAC_evaluations':small_turn,'model_feedback_teacher_evaluations':teacher,
            'connected_SAC_prefix_and_saved_teacher':connected,
            'connected_SAC_prefix_learned_correction_and_position_hold':learned_connections,
            'packaged_controller_free_model_trials':packaged,
            'correction_behavior_cloning_runs_not_SAC_environment_steps':corrections,
             'SAC_step_count_is_completed_requested_budget_not_exact_trainer_counter':True,
             'old_CCD10_passes_do_not_certify_CCD50':True,
            'learned_connection_cases_were_also_used_for_teacher_data':True,
            'repeated_nominal_certificate_run_is_not_an_independent_success_condition':True,
            'second90_rotation_passed':False,'physical_successes':0,
            'distillation_completed':False,'hardware_execution_allowed':False}
    save_json(OUT/'evaluation_summary.json',result)
    print(json.dumps({'evaluation_rows':len(rows),'training_runs':len(training),
                      'new_training_steps':result['new_training_steps'],'second90_rotation_passed':False}),flush=True)


if __name__=='__main__':main()
