import json
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/directional_contact_research'
def main():
 notes=[]
 for f in sorted(O.glob('normal_gap_cap_*/review.json')):
  r=json.loads(f.read_bytes());notes.append(dict(source=f.relative_to(R).as_posix(),nominal_inverse_deg=r['modeled_inverse_at_hold_deg'],safety_passed=r['passed_unchanged_safety_bounds'],rotation_goal_passed=r['target_rotation_met'],hardware_execution_allowed=False))
 for name in ('opposed_contact_ik','opposed_contact_free_center_ik'):
  f=O/name/'geometry_review.json';r=json.loads(f.read_bytes());notes.append(dict(source=f.relative_to(R).as_posix(),geometry_completed_90=r['geometry_completed_90'],knots_completed=r['knots_completed'],failed=r['failed'],hardware_execution_allowed=False))
 for folder in ('full_reverse_stroke_research','whole_forward_reverse_research'):
  f=R/'right_hand_rotation_20261030'/folder/'nominal/review.json';r=json.loads(f.read_bytes());c=r['cases'][0];notes.append(dict(source=f.relative_to(R).as_posix(),nominal_inverse_deg=c['inverse_from_forward_hold_deg'],safety_passed=c['passed'],rotation_goal_passed=False,hardware_execution_allowed=False))
 out=dict(status='counterturn_research_hypotheses_not_admitted_for_execution',studies=notes,current_measured_physical_inverse_deg=-42.311878838667035,inverse90_verified=False,
   next_research='Direction-specific rolling/regrasp contacts; normal-only corrections and reversal alone did not produce sufficient free-body rotation.',
   current_operator_handoff='right_hand_rotation_20261030/single_forward/handoff.json',current_operator_scope='Exact already observed single forward and automatic OPEN; repeatability observation only.',devices_opened_by_studies=False,assistant_motion_commands=0)
 (O/'progress.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
 for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
  f=R/name;r=json.loads(f.read_bytes());r.update(current_motor_connection='All16 OFF/error0/watchdog0; three identical fresh snapshots at05:14, <=1tick difference from reviewed canonical start.',source_off_drift_ticks=0,latest_camera_inventory='contact_view_readonly_20261030_045210_021307/review.json',expected_camera_present=True,inverse_research_record=(O/'progress.json').relative_to(R).as_posix());f.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
 f=R/'right_hand_rotation_20261030/single_forward/operator_guide.html';f.write_text(f.read_text(encoding='utf-8').replace(' · ',' / '),encoding='utf-8')
 print(json.dumps(dict(research_studies_recorded=len(notes),new_counterturn_execution_admissions=0,next_operator_command='leap_single_forward_trial.cmd',assistant_motion_commands=0)))
if __name__=='__main__':main()
