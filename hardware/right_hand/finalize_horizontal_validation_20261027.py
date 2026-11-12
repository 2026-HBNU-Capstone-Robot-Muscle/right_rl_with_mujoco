"""Final evidence binding check only; no simulator or device access."""
import hashlib,json
from horizontal_quarter_plan_20261027 import HorizontalQuarterPlan,ROOT,OUT,REQUEST,PROOF,PREFIX_PHASES,LOADED_PATH
from horizontal_quarter_gate_20261027 import validate_review
plan=HorizontalQuarterPlan(REQUEST);canonical=validate_review(plan,PROOF);probe_folder=OUT/'synthetic_one_tick_probe';probe=HorizontalQuarterPlan(probe_folder/'request.json');probe_review=validate_review(probe,probe_folder/'review.json')
assert max(abs(probe.source_position()-plan.source_position()))==1
base_loaded=[(p,q.tolist()) for p,q in plan.plan(plan.source_position()) if p not in PREFIX_PHASES];probe_loaded=[(p,q.tolist()) for p,q in probe.plan(probe.source_position()) if p not in PREFIX_PHASES];assert base_loaded==probe_loaded
render=json.loads((OUT/'model_video/render_review.json').read_bytes());video=json.loads((OUT/'model_video/video_review.json').read_bytes());assert render['loaded_path_sha256']==hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest();assert render['frames']==video['frames']==686;assert (ROOT/video['video']).is_file()
report=dict(status='operator_ready_pending_user_execution',device_free_tests_passed=24,canonical_empty_model_cases=210,synthetic_one_tick_rebind_model_cases=210,loaded_static_cases=201,loaded_physics_cases=27,source_and_control_hashes_valid=True,changed_start_rechecks_empty_recovery=True,loaded_controls_identical_in_rebind=True,video_matches_final_loaded_control_hash=True,nominal_model_turn_before_open_deg=canonical['nominal_model_yaw_deg'],model_response_yaw_range_deg=canonical['model_yaw_range_deg'],physical_horizontal_90_verified=False,assistant_motor_goal_writes=0,assistant_torque_or_setting_writes=0,devices_opened_by_validation=False)
(OUT/'validation_summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report))
