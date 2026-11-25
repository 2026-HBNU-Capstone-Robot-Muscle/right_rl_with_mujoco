"""Finalize existing model results, never starts hardware or simulations."""
import json,hashlib,argparse
from retained_slow_yaw_plan_20261029 import RetainedSlowYawPlan,OUT,REQUEST,PROOF,ROOT,hashes
from retained_slow_yaw_gate_20261029 import make_review,validate_review
parser=argparse.ArgumentParser();parser.add_argument('--empty-review',default='empty_recovery_screen/review.json');parser.add_argument('--reference-only',action='store_true');args=parser.parse_args()
p=RetainedSlowYawPlan(REQUEST);empty=json.loads((OUT/args.empty_review).read_bytes())
r=make_review(p,empty,hashes())
PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');validate_review(p,PROOF)
ready=dict(status='offline_reference_review_passed_waiting_for_cube_removal_and_fresh_OFF_readings' if args.reference_only else 'ready_for_one_user_started_retained_red_slow_yaw_trial',command='leap_retained_slow_yaw.cmd',
    reference_only=args.reference_only,fresh_current_empty_hand_review_pending=args.reference_only,
    request_sha256=hashlib.sha256(REQUEST.read_bytes()).hexdigest(),review_sha256=hashlib.sha256(PROOF.read_bytes()).hexdigest(),
    supervised_experiment_only=True,robust90_degree_motion_verified=False,nominal_motion_seconds=r['nominal_motion_seconds'],
    fresh_empty_static_cases=201,fresh_empty_dynamics_cases=9,isolated_hand_dynamics_cases=9,
    read_recovery_policy=r['read_resynchronization_policy'],fast_health_read_evidence=r['fast_health_read_evidence'],initial_orientation='RED up; GREEN toward wrist ID41',
    placement_reference='pitch_slow_yaw_20261029_004532_520638/cube_transfer_hold.jpg',assistant_motor_writes=0)
(OUT/'readiness.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(ready,ensure_ascii=False))
