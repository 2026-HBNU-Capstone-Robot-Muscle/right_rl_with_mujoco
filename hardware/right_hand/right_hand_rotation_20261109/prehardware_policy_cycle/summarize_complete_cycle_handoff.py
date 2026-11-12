"""Summarize the verified model cycle and finite comparisons for handoff."""
from pathlib import Path
import hashlib
import json
import numpy as np

OUT=Path(__file__).resolve().parent;BASE=OUT/'next_cycle/handoff_refinement_20261110';FOLDER=BASE/'reposition_then_second_rotation_20261111'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    parent=BASE/'robust_mu025_complete_cycle75ms_replay_v39';r=json.loads((parent/'review.json').read_text(encoding='utf-8'));assert r['full_two_turn_two_hold_success'] and not r['stop']
    first=BASE/'robust_cycle_preparation_20261111/low_friction_first_hold_state_v4/end_state.npz';end=parent/'end_state.npz';C0=np.load(first,allow_pickle=False)['qpos'][-7:-4];C1=np.load(end,allow_pickle=False)['qpos'][-7:-4];delta=(C1-C0)*1000;distance_discrepancy=abs(float(np.linalg.norm(delta))-r['first_station_error_mm']);assert distance_discrepancy<.001,(np.linalg.norm(delta),r['first_station_error_mm'])
    finite=[];sources={p.relative_to(OUT).as_posix():sha(p) for p in (parent/'review.json',parent/'research_goals.npz',first,end)}
    for case in ('mu024','mu026','xplus1'):
        path=BASE/('complete_cycle_variation_'+case)/'review.json'
        if path.exists():
            trial=json.loads(path.read_text(encoding='utf-8'));sources[path.relative_to(OUT).as_posix()]=sha(path);finite.append(dict(case=case,report_available=True,steps=trial['steps'],stop=trial['stop'],sliding_friction_hypothesis=trial['sliding_friction_hypothesis'],initial_cube_worldX_offset_mm=trial['initial_cube_worldX_offset_mm'],full_two_turn_two_hold_success=trial['full_two_turn_two_hold_success'],first_hold=trial['first_hold'],final_hold=trial['final_hold'],final180_error_deg=trial['final180_error_deg']))
        else:finite.append(dict(case=case,report_available=False,report_path=path.relative_to(OUT).as_posix()))
    result=dict(status='specific_low_friction_model_cycle_verified',source_files_sha256=sources,steps=r['steps'],simulated_seconds=r['all1ms_samples']/1000,control_seconds=.075,max_goal_ticks=2,actual_state_resets_after_initial_placement=0,first_hold=r['first_hold'],final_hold=r['final_hold'],cube_center_world_model_first_m=C0.tolist(),cube_center_world_model_final_m=C1.tolist(),cube_center_world_model_delta_mm=delta.tolist(),center_displacement_mm=float(np.linalg.norm(delta)),qpos_to_reported_geom_distance_discrepancy_mm=distance_discrepancy,first_hold_direction_margin_to5deg=5-r['first_hold']['orientation_error_deg'],final_upper_margin_above0p3mm=r['final_hold']['minimum_upper_lower_mm']-.3,finite_variations=finite,finite_checks_are_saved_path_tests_not_new_adaptive_policy=True,archived_learned_policy_derived_prefix_goals=1700,remaining_privileged_model_teacher_goals=r['steps']-1700,full_replay_is_stored_goal_execution=True,left_hand_reach_load_and_physical_coordinate_registration_unverified=True,new_SAC_training_steps=0,new_distillation_completed=False,devices_opened=0,remote_writes=0,hardware_execution_allowed=False)
    (FOLDER/'complete_cycle_handoff.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(dict(steps=r['steps'],model_center_displacement_mm=result['center_displacement_mm'],finite=[dict(case=t['case'],report_available=t['report_available'],passed=t.get('full_two_turn_two_hold_success')) for t in finite])),flush=True)

if __name__=='__main__':main()
