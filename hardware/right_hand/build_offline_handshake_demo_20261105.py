"""One simulated coordination cycle, followed by a deliberately blocked seam."""
import json,hashlib
from pathlib import Path
from offline_two_hand_coordinator_20261105 import Coordinator,CyclePlan,Phase,RightHealth,FutureLiveEvidence
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip';candidate=O/'recentered_free/i-77_m-16_t-56/from_reference/candidate.npz';review=json.loads((O/'certificate_review_v2.json').read_bytes());sha=hashlib.sha256(candidate.read_bytes()).hexdigest();assert review['candidate_sha256']==sha
plan=CyclePlan('placement_forward_common_grip_v2',sha,'common_cube_frame_not_yet_physically_registered',90,{Phase.RIGHT.value:300.,Phase.LEFT_APPROACH.value:30.,Phase.LEFT_TURN.value:30.,Phase.LEFT_RETREAT.value:30.},'placed_open','common_body_grip_v2')
c=Coordinator(initial_grip_id='placed_open');actions=[]
def add(values):
    from dataclasses import asdict
    actions.extend(asdict(v) for v in values)
def health(t):return RightHealth(t,frozenset(range(1,17)),True,True,True,True)
add(c.start(plan,0.,left_clear_confirmed=True));add(c.receive(c.pending,'rotation_complete_hold_ready',131.,right_health=health(131.)));add(c.receive(c.pending,'at_turn_start',133.,right_health=health(133.)));add(c.receive(c.pending,'layer_turn_complete',135.,right_health=health(135.)));c.receive(c.pending,'clear_of_cube',137.,left_clear_confirmed=True);blocked=c.start(plan,138.,left_clear_confirmed=True);assert blocked==[]
e=FutureLiveEvidence(offline_model_review_passed=review['certificate_based_task_passed']==27)
report=dict(status='simulated_integration_handshake_only',candidate_sha256=sha,events=c.history,intents=actions,final_phase=c.phase.value,right_still_holding=True,reusing_the_placement_entry_path_for_next_cycle_blocked=True,required_next_entry_grip='common_body_grip_v2',next_loaded_primitive_not_yet_available=True,hardware_commands_sent=0,transport_adapters=0,all_acknowledgements_and_health_in_this_demo_are_synthetic=True,example_left_time_budgets_are_not_production_approval=True,live_readiness=e.live_ready(),missing_live_evidence=e.missing(),devices_opened=False)
(O/'integration_dry_run.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k not in ('events','intents')}))
