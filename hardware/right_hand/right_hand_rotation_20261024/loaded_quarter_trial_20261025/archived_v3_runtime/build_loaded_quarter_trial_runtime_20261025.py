"""Create an explicitly experimental variant without editing old runners."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def once(s,a,b):
    if s.count(a)!=1:raise ValueError('Ambiguous generator block: '+a[:90])
    return s.replace(a,b)
def main():
    source=ROOT/'screen_reverse_quarter_resume_empty_20261025.py';screen=source.read_text(encoding='utf-8')
    old='from reverse_quarter_resume_empty_plan_20261025 import ReverseQuarterResumeEmptyPlan as BookGraspPlan,extra_hashes,resume_goals as motion_goals,EXTRA_RING_PAIR,EXTRA_RING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE'
    screen=once(screen,old,'from loaded_quarter_trial_plan_20261025 import LoadedQuarterTrialPlan as BookGraspPlan,extra_hashes,motion_goals,START_HOLD,TURN_PHASE,END_HOLD,RETURN_PHASE,EXTRA_RING_PAIR,EXTRA_RING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE,validate_experiment_basis,THUMB_NEIGHBOR_PAIR,THUMB_NEIGHBOR_LIMIT_MM,INITIAL_SELF_SCOPE\nSELF_PAIRS=SELF_PAIRS+(THUMB_NEIGHBOR_PAIR,)')
    screen=screen.replace('if self_pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM or self_pairs.get(EXTRA_RING_PAIR,0.)>EXTRA_RING_INITIAL_LIMIT_MM:',
        'if self_pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM or self_pairs.get(EXTRA_RING_PAIR,0.)>EXTRA_RING_INITIAL_LIMIT_MM or self_pairs.get(THUMB_NEIGHBOR_PAIR,0.)>THUMB_NEIGHBOR_LIMIT_MM:')
    screen=once(screen,'        rows=paths[0][\'rows\'];angles=paths[0][\'angles\'];lift_s=paths[0][\'lift_count\']*.05',
        "        self_envelope={}\n        for case in static:\n            if case['book_front_mm']==front:\n                for pair,depth in case['initial_self_contacts_mm'].items():self_envelope[pair]=max(self_envelope.get(pair,0.),depth)\n        rows=paths[0]['rows'];angles=paths[0]['angles'];lift_s=paths[0]['lift_count']*.05")
    screen=once(screen,'self_escape=ContactEscape(self_pairs,SELF_PAIRS,MAX_INITIAL_THUMB_SELF_MM,self_deadline,monotonic=False)',
        'nominal_initial_self=dict(self_pairs)\n            self_escape=ContactEscape(self_envelope,SELF_PAIRS,MAX_INITIAL_THUMB_SELF_MM,self_deadline,monotonic=False)')
    screen=once(screen,'nominal_initial_support_contacts_mm=nominal_initial_support,passive_enable_hold_support=hold_evidence,',
        'nominal_initial_support_contacts_mm=nominal_initial_support,nominal_initial_self_contacts_mm=nominal_initial_self,passive_enable_hold_support=hold_evidence,')
    screen=once(screen,'ring_clear_complete_at=ring_clear_at,',
        "ring_clear_complete_at=ring_clear_at,thumb_tip_clear_goal_ticks=(int((raw+path['offset'])[0]-2*self_escape.complete_at) if self_escape.complete_at is not None else None),")
    screen=once(screen,'initial_ring_neighbor_limit_mm=EXTRA_RING_INITIAL_LIMIT_MM,',
        'initial_ring_neighbor_limit_mm=EXTRA_RING_INITIAL_LIMIT_MM,initial_thumb_neighbor_limit_mm=THUMB_NEIGHBOR_LIMIT_MM,initial_self_scope=INITIAL_SELF_SCOPE,')
    screen=once(screen,"if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:\n        raise ValueError('Whole EMPTY path must not include placement')",
        "if not plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE!=TURN_PHASE:\n        raise ValueError('Loaded trial requires explicit placement checkpoint')")
    screen=once(screen,'        for (phase,q),goal in zip(rows,angles[1:]):\n            dynamics.append((phase,goal))',
        "        for (phase,q),goal in zip(rows,angles[1:]):\n            if phase==TURN_PHASE and not inserted:\n                dynamics.extend([('native_placement_hold',dynamics[-1][1])]*80);inserted=True\n            dynamics.append((phase,goal))")
    for a,b in (
        ('accepted_for_supervised_grasp_approach=False','accepted_for_supervised_grasp_approach=accepted'),
        ('empty_hand_only=True','empty_hand_only=False'),
        ('full_quarter_reference_path=False,complete_recovery_and_suffix_checked=True','full_quarter_reference_path=True,full_empty_return_and_loaded_reference_hand_route_checked=True'),
        ('interrupted_suffix_verified=True','interrupted_suffix_verified=False,experimental_loaded_trial=True,experimental_model_review=validate_experiment_basis()'),
        ('placement_checkpoint_required=False','placement_checkpoint_required=True'),
        ('native_placement_hold_samples=0','native_placement_hold_samples=80'),
        ('Goals after PLACE are fixed.','Targets after PLACE follow the complete proposed reference. The Cube is not present in this hand-route screen.'),
        ('Whole EMPTY quarter-turn reference once. No placement or loaded Cube turn.','Complete EMPTY return and proposed loaded reference hand route. Separate conditional Cube diagnostic required; actual loaded response unverified.'),
    ):screen=once(screen,a,b)
    (ROOT/'screen_loaded_quarter_trial_20261025.py').write_text(screen,encoding='utf-8')
    gate=(ROOT/'reverse_quarter_resume_empty_gate_20261025.py').read_text(encoding='utf-8')
    gate=once(gate,'from reverse_quarter_resume_empty_plan_20261025 import extra_hashes,START_HOLD,TURN_PHASE,resume_goals as motion_goals,EXTRA_RING_PAIR,EXTRA_RING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE',
        'from loaded_quarter_trial_plan_20261025 import extra_hashes,START_HOLD,TURN_PHASE,RETURN_PHASE,motion_goals,empty_return_goals,EXTRA_RING_PAIR,EXTRA_RING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE,validate_experiment_basis,THUMB_NEIGHBOR_PAIR,THUMB_NEIGHBOR_LIMIT_MM,INITIAL_SELF_SCOPE')
    gate=once(gate,'SELF_PAIRS=SELF_PAIRS+(CASING_PAIR,EXTRA_RING_PAIR)', 'SELF_PAIRS=SELF_PAIRS+(CASING_PAIR,EXTRA_RING_PAIR,THUMB_NEIGHBOR_PAIR)')
    gate=once(gate,"pairs.get(EXTRA_RING_PAIR,0.)>EXTRA_RING_INITIAL_LIMIT_MM)", "pairs.get(EXTRA_RING_PAIR,0.)>EXTRA_RING_INITIAL_LIMIT_MM or pairs.get(THUMB_NEIGHBOR_PAIR,0.)>THUMB_NEIGHBOR_LIMIT_MM)")
    gate=once(gate,"or r.get('initial_ring_neighbor_limit_mm')!=EXTRA_RING_INITIAL_LIMIT_MM", "or r.get('initial_ring_neighbor_limit_mm')!=EXTRA_RING_INITIAL_LIMIT_MM or r.get('initial_thumb_neighbor_limit_mm')!=THUMB_NEIGHBOR_LIMIT_MM or r.get('initial_self_scope')!=INITIAL_SELF_SCOPE")
    gate=once(gate,'        threshold=s.get(\'thumb_clear_goal_ticks\')',
        "        expected_tip=raw[0]+s['offset_ticks'][0]-2*s['self_escape_complete_at']\n        if s.get('thumb_tip_clear_goal_ticks')!=expected_tip:return False\n        threshold=s.get('thumb_clear_goal_ticks')")
    gate=once(gate,'    expected=sorted(tuple(v.tolist()) for v in offsets())',
        "    def initial_self_valid(s,front):\n        bound={}\n        for c in static:\n            if c['book_front_mm']==front:\n                for pair,depth in c['initial_self_contacts_mm'].items():bound[pair]=max(bound.get(pair,0.),depth)\n        nominal=next(c['initial_self_contacts_mm'] for c in static if c['book_front_mm']==front and c['offset_ticks']==[0]*16)\n        return s.get('initial_self_contacts_mm')==bound and s.get('nominal_initial_self_contacts_mm')==nominal\n    expected=sorted(tuple(v.tolist()) for v in offsets())")
    gate=once(gate,'or not all(passive_hold_valid(s,front) for s in dr)', 'or not all(passive_hold_valid(s,front) for s in dr) or not all(initial_self_valid(s,front) for s in dr)')
    gate=once(gate,"checker=ROOT/'screen_reverse_quarter_resume_empty_20261025.py'","checker=ROOT/'screen_loaded_quarter_trial_20261025.py'")
    gate=once(gate,"if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:\n        raise RuntimeError('Unexpected placement in EMPTY path; no writes')",
        "if not plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE!=TURN_PHASE:\n        raise RuntimeError('Loaded experiment requires explicit PLACE/REMOVE checkpoints; no writes')\n    anchor=int(np.argmin(np.max(abs(motion_goals()-raw),axis=1)))\n    actual_return=np.array([q for p,q in rows if p==RETURN_PHASE])\n    expected_return=empty_return_goals(anchor)[1:]\n    if actual_return.size==0:actual_return=np.empty((0,16),dtype=int)\n    if not np.array_equal(actual_return,expected_return):raise RuntimeError('EMPTY return differs from the derived tip-clearance reference; no writes')")
    for a,b in (
        ("'accepted_for_supervised_grasp_approach':False","'accepted_for_supervised_grasp_approach':True"),
        ("'empty_hand_only':True","'empty_hand_only':False"),
        ("'full_quarter_reference_path':False,'complete_recovery_and_suffix_checked':True","'full_quarter_reference_path':True,'full_empty_return_and_loaded_reference_hand_route_checked':True"),
        ("'interrupted_suffix_verified':True","'interrupted_suffix_verified':False,'experimental_loaded_trial':True"),
        ("'placement_checkpoint_required':False","'placement_checkpoint_required':True"),
        ("r.get('native_placement_hold_samples')!=0","r.get('native_placement_hold_samples')!=80 or r.get('experimental_model_review')!=validate_experiment_basis()"),
    ):gate=once(gate,a,b)
    (ROOT/'loaded_quarter_trial_gate_20261025.py').write_text(gate,encoding='utf-8')
    from loaded_quarter_trial_plan_20261025 import LoadedQuarterTrialPlan,validate_experiment_basis,RETURN_PHASE
    folder=ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/fixture_v3';folder.mkdir(parents=True,exist_ok=True)
    request=json.loads((ROOT/'completed_empty_readonly_20261025_091531_043679/request.json').read_bytes())
    request['plan_version']=LoadedQuarterTrialPlan.PLAN_VERSION
    (folder/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
    p=LoadedQuarterTrialPlan(folder/'request.json');rows=p.plan(p.source_position());validate_experiment_basis()
    print('GOALS',len(rows),'ON_SECONDS',(len(rows)+20)*.05,'RETURN_GOALS',sum(n==RETURN_PHASE for n,q in rows))
if __name__=='__main__':main()
