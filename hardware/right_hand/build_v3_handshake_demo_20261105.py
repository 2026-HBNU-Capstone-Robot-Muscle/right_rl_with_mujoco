"""A source-bound simulated exchange only; failed repeat-cycle seam stays blocked."""
import json,hashlib
from pathlib import Path
from dataclasses import asdict
from offline_two_hand_coordinator_20261105 import Coordinator,CyclePlan,Phase,RightHealth,FutureLiveEvidence
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip';candidate=O/'tip_squeeze/id1/plus12/candidate.npz';review=json.loads((O/'certificate_review_v3_thumb12.json').read_bytes());sha=hashlib.sha256(candidate.read_bytes()).hexdigest()
assert review['candidate_sha256']==sha and review['model_cases']==27
plan=CyclePlan('placement_forward_model61_bodygrip_v3',sha,'common_cube_frame_not_yet_physically_registered',90,{Phase.RIGHT.value:300.,Phase.LEFT_APPROACH.value:30.,Phase.LEFT_TURN.value:30.,Phase.LEFT_RETREAT.value:30.},'placed_open','common_body_grip_v3_thumb12')
c=Coordinator(initial_grip_id='placed_open');actions=[]
def add(items):actions.extend(asdict(v) for v in items)
def health(t):return RightHealth(t,frozenset(range(1,17)),True,True,True,True)
add(c.start(plan,0.,left_clear_confirmed=True));add(c.receive(c.pending,'rotation_complete_hold_ready',131.,right_health=health(131.)));add(c.receive(c.pending,'at_turn_start',133.,right_health=health(133.)));add(c.receive(c.pending,'layer_turn_complete',135.,right_health=health(135.)));c.receive(c.pending,'clear_of_cube',137.,left_clear_confirmed=True)
blocked=c.start(plan,138.,left_clear_confirmed=True);assert blocked==[] and c.phase==Phase.COMPLETE
e=FutureLiveEvidence(offline_model_review_passed=review['certificate_based_task_passed']==27)
report=dict(status='simulated_v3_integration_exchange_only',candidate_sha256=sha,events=c.history,intents=actions,final_phase=c.phase.value,right_still_holding_in_synthetic_demo=True,reusing_placement_entry_path_for_next_cycle_blocked=True,required_next_entry_grip='common_body_grip_v3_thumb12',next_loaded_primitive_not_validated=True,physical_target90_not_verified=True,upper_layer_turn_is_only_a_synthetic_acknowledgement=True,all_acknowledgements_and_health_are_synthetic=True,example_time_budgets_not_production_approval=True,hardware_commands_sent=0,transport_adapters=0,live_readiness=e.live_ready(),missing_live_evidence=e.missing(),devices_opened=False)
(O/'integration_dry_run_v3.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k not in ('events','intents')}),flush=True)
