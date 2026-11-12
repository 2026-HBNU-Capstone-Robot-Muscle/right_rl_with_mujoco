"""Record the user's common-grip-position requirement as a design objective."""
import json
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip'
def main():
    comparison=json.loads((O/'seating_comparison.json').read_bytes());goal=json.loads((O/'goal_specification.json').read_bytes())
    summary=dict(status='common_terminal_grip_design_goal_initial_support_filter_complete',user_intent='Keep the holding location consistent regardless of whole-cube reorientation.',
        desired_terminal_invariants=['cube center relative to palm','top-layer height','cube tilt and edge alignment'],during_rotation_center_need_not_be_exactly_constant=True,
        preferred_first_hypothesis='Palm-supported cube near the finger roots, laterally toward the index/middle region; grip the lower body and leave the upper-layer sweep clear.',
        candidate_preference_is_an_engineering_hypothesis_not_a_physical_trial_result=True,
        initial_support_counts={v['label']:v['support_passed'] for v in comparison['candidates']},
        comparison_scope='4 starting-center hypotheses x9 gain/friction cases; same open hand and finite actuators. Not a comparison of complete grasp trajectories or lifting between fingertips.',
        no_initial_support_winner_established=True,terminal_metric_target_selected=False,automatic_real_cube_repositioning_implemented=False,physical_model_registration_verified=False,
        remaining=['Find a lower-body contact arrangement that clears the whole upper-layer sweep','Check a continuous free-cube transition to the same terminal pose after each needed reorientation','Verify terminal pose reproduction physically','Verify actual left-hand layer-turn load after integration'],
        no_new_user_hand_or_cube_repositioning_requested=True,hardware_execution_allowed=False,assistant_motor_writes=0)
    (O/'design_intent.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    for rel in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        p=R/rel;d=json.loads(p.read_bytes());d.update(canonical_terminal_grip_required=True,latest_canonical_grip_design=(O/'design_intent.json').relative_to(R).as_posix(),canonical_terminal_position_selected=False,next_user_action_required=False,next_user_command=None,next_user_command_ready=False);p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
