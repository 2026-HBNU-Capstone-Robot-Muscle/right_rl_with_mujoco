"""Create a separate photo-only gate, preserving the prior EMPTY entry gate."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def once(s,a,b):
    if s.count(a)!=1:raise ValueError('Template is not unique: '+a[:70])
    return s.replace(a,b)

def main():
    screen=(ROOT/'screen_reverse_start_empty_20261025.py').read_text(encoding='utf-8')
    screen=once(screen,'LOADED_PHASE=END_HOLD',
        'from reverse_place_capture_plan_20261025 import ReversePlacePlan as BookGraspPlan,extra_hashes,BASE_RELEASE,CASING_PAIR,CASING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE\n'
        'THUMB_RELEASE=BASE_RELEASE\nSELF_PAIRS=SELF_PAIRS+(CASING_PAIR,)\nLOADED_PHASE=END_HOLD')
    screen=once(screen,'pairs,self_pairs=inspect(d)\n            releases=',
        "pairs,self_pairs=inspect(d)\n            if self_pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM:\n"
        "                raise ValueError('OFF casing contact exceeds the dedicated 1mm model bound')\n            releases=")
    screen=once(screen,'pairs,self_pairs=inspect(d);deadline=.5+ring_s+tip_s+',
        "pairs,self_pairs=inspect(d)\n            if self_pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM:\n"
        "                raise ValueError('OFF casing contact exceeds the dedicated 1mm model bound')\n            deadline=.5+ring_s+tip_s+")
    screen=once(screen,"thumb_clear_goal_ticks=(int((raw+path['offset'])[0]-2*max(0,self_escape.complete_at-path['ring_count']))",
        "thumb_clear_goal_ticks=(int((raw+path['offset'])[3]-2*max(0,self_escape.complete_at-path['ring_count']))")
    screen=once(screen,"rows=paths[0]['rows'];angles=paths[0]['angles'];lift_s=paths[0]['lift_count']*.05",
        "support_envelope={}\n        for case in static:\n            if case['book_front_mm']==front:\n"
        "                for pair,depth in case['initial_support_contacts_mm'].items():\n"
        "                    support_envelope[pair]=max(support_envelope.get(pair,0.),depth)\n"
        "        rows=paths[0]['rows'];angles=paths[0]['angles'];lift_s=paths[0]['lift_count']*.05")
    screen=once(screen,'escape=ContactEscape(pairs,ALLOWED_SUPPORT_PAIRS,MAX_INITIAL_SUPPORT_MM,deadline,monotonic=False)',
        "nominal_initial_support=dict(pairs)\n            initial_bound=dict(support_envelope);hold_contacts=dict(pairs)\n"
        "            probe=mujoco.MjData(m);pose(probe,angles[0]);probe.ctrl[:]=angles[0]\n"
        "            for _ in range(round(.5/m.opt.timestep)):\n"
        "                mujoco.mj_step(m,probe);probe_pairs,_=inspect(probe)\n"
        "                for pair,depth in probe_pairs.items():hold_contacts[pair]=max(hold_contacts.get(pair,0.),depth)\n"
        "            for pair,depth in hold_contacts.items():\n"
        "                if pair not in initial_bound:\n"
        "                    if depth>.15:raise ValueError('New passive enable support contact exceeds the existing 0.15mm model allowance')\n"
        "                    initial_bound[pair]=depth\n"
        "            hold_evidence=dict(duration_seconds=.5,goal_ticks=raw.tolist(),contact_maximums_mm=hold_contacts,new_pair_depth_limit_mm=.15)\n"
        "            escape=ContactEscape(initial_bound,ALLOWED_SUPPORT_PAIRS,MAX_INITIAL_SUPPORT_MM,deadline,monotonic=False)")
    screen=once(screen,'row=dict(book_front_mm=front,gain_scale=scale,finite=finite,max_tracking_rad=tracking,',
        'row=dict(book_front_mm=front,gain_scale=scale,finite=finite,max_tracking_rad=tracking,\n'
        '                nominal_initial_support_contacts_mm=nominal_initial_support,passive_enable_hold_support=hold_evidence,')
    screen=once(screen,'empty_hand_only=True,full_CAD_tip_contacts=True,cube_disabled_entire_screen=True,',
        'empty_hand_only=False,placement_capture_only=True,joint_goals_fixed_after_placement=True,\n'
        '        casing_release_motor_id=4,initial_casing_contact_limit_mm=CASING_INITIAL_LIMIT_MM,\n'
        '        initial_support_scope=INITIAL_SUPPORT_SCOPE,no_contact_outside_reviewed_initial_envelope=True,\n'
        '        full_CAD_tip_contacts=True,cube_disabled_entire_screen=True,')
    screen=once(screen,'cube_contact_verified=False,placement_checkpoint_required=False,','cube_contact_verified=False,placement_checkpoint_required=True,')
    screen=once(screen,'plan=plan_type(request);raw=plan.source_position()',
        "plan=plan_type(request);raw=plan.source_position()\n    candidate_rows=plan.plan(raw)\n"
        "    checkpoint=next(i for i,(phase,_) in enumerate(candidate_rows) if phase==plan.CAPTURE_BEFORE_PHASE)\n"
        "    if checkpoint==0 or any(not np.array_equal(q,candidate_rows[checkpoint-1][1]) for _,q in candidate_rows[checkpoint:]):\n"
        "        raise ValueError('Placement observation contains moving joint goals')")
    screen=screen.replace('EMPTY-only entry, no cube placement checkpoint, no loaded motion or live policy.',
        'Photo-only placement checkpoint. All joint goals remain fixed after placement. No rotation or live policy.')
    screen=once(screen,'The cube is disabled in this hand clearance screen. It does not certify cube\ncontact, placement, grip force or rotation. Physical placement is an explicit\noperator checkpoint, and the only loaded targets are the reached old approach.',
        'The cube is disabled in this hand clearance screen. It does not certify cube\n'
        'contact, placement, grip force or rotation. Goals after PLACE are fixed.\n'
        'Initial support bounds distinguish the +/-4 tick start and fixed enable hold;\n'
        'contact forces and hardware current/voltage/tracking stops are unchanged.')
    (ROOT/'screen_reverse_place_capture_20261025.py').write_text(screen,encoding='utf-8')
    gate=(ROOT/'reverse_start_empty_gate_20261025.py').read_text(encoding='utf-8')
    gate=once(gate,'from reverse_start_empty_plan_20261025 import extra_hashes','from reverse_place_capture_plan_20261025 import extra_hashes')
    gate=once(gate,'def screen_failure_details(path):',
        'from reverse_place_capture_plan_20261025 import BASE_RELEASE,CASING_PAIR,CASING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE\n'
        'THUMB_RELEASE=BASE_RELEASE\nSELF_PAIRS=SELF_PAIRS+(CASING_PAIR,)\n\n\ndef screen_failure_details(path):')
    gate=once(gate,"checker=ROOT/'screen_reverse_start_empty_20261025.py'","checker=ROOT/'screen_reverse_place_capture_20261025.py'")
    gate=once(gate,"'empty_hand_only':True,'full_CAD_tip_contacts':True,",
        "'empty_hand_only':False,'placement_capture_only':True,'joint_goals_fixed_after_placement':True,\n"
        "        'casing_release_motor_id':4,'no_contact_outside_reviewed_initial_envelope':True,'full_CAD_tip_contacts':True,")
    gate=once(gate,"or r.get('actual_tracking_stop_rad')!=.18", "or r.get('actual_tracking_stop_rad')!=.18\n"
        "        or r.get('initial_casing_contact_limit_mm')!=CASING_INITIAL_LIMIT_MM or r.get('initial_support_scope')!=INITIAL_SUPPORT_SCOPE")
    gate=once(gate,"if not isinstance(pairs,dict) or not set(pairs).issubset(allowed):return False",
        "if not isinstance(pairs,dict) or not set(pairs).issubset(allowed):return False\n"
        "        if kind=='self' and pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM:return False")
    gate=once(gate,"def contact_valid(s,kind,dynamic):",
        "counts_cache={}\n    def phase_counts(s,dynamic):\n"
        "        if dynamic:return ring_count,tip_count,lift_count\n"
        "        key=tuple(s['offset_ticks'])\n        if key not in counts_cache:\n"
        "            case_rows=plan.plan(raw+np.asarray(key))\n"
        "            counts_cache[key]=tuple(sum(p==phase for p,q in case_rows) for phase in (RING_RELEASE,THUMB_RELEASE,LIFT))\n"
        "        return counts_cache[key]\n    def contact_valid(s,kind,dynamic):\n"
        "        ring_count,tip_count,lift_count=phase_counts(s,dynamic)")
    gate=once(gate,"step=s.get('max_goal_step_ticks',float('nan'))",
        "ring_count,_,_=phase_counts(s,False)\n        step=s.get('max_goal_step_ticks',float('nan'))")
    gate=once(gate,"expected_threshold=raw[0]+s['offset_ticks'][0]-2*max(0,s['self_escape_complete_at']-ring_count)",
        "expected_threshold=raw[3]+s['offset_ticks'][3]-2*max(0,s['self_escape_complete_at']-ring_count)")
    gate=once(gate,"expected=sorted(tuple(v.tolist()) for v in offsets())",
        "def envelope_bound(front):\n        envelope={}\n        for s in static:\n            if s.get('book_front_mm')==front:\n"
        "                for pair,depth in s['initial_support_contacts_mm'].items():envelope[pair]=max(envelope.get(pair,0.),depth)\n"
        "        return envelope\n"
        "    def passive_hold_valid(s,front):\n        bound=envelope_bound(front);e=s.get('passive_enable_hold_support',{})\n"
        "        if e.get('duration_seconds')!=.5 or e.get('goal_ticks')!=raw.tolist() or e.get('new_pair_depth_limit_mm')!=.15:return False\n"
        "        pairs=e.get('contact_maximums_mm')\n"
        "        if not isinstance(pairs,dict) or not set(pairs).issubset(ALLOWED_SUPPORT_PAIRS):return False\n"
        "        for pair,depth in pairs.items():\n"
        "            if not np.isfinite(depth) or depth<0 or depth>(bound[pair] if pair in bound else .15)+1e-6:return False\n"
        "            if pair not in bound:bound[pair]=depth\n"
        "        return s.get('initial_support_contacts_mm')==bound\n"
        "    expected=sorted(tuple(v.tolist()) for v in offsets())")
    gate=once(gate,"or not all(valid(s,False) for s in sr) or not all(valid(s,True) for s in dr)):",
        "or not all(valid(s,False) for s in sr) or not all(valid(s,True) for s in dr)\n"
        "            or not all(passive_hold_valid(s,front) for s in dr)\n"
        "            or any(s.get('nominal_initial_support_contacts_mm')!=next(c['initial_support_contacts_mm'] for c in sr if c['offset_ticks']==[0]*16) for s in dr)):")
    gate=once(gate,"'cube_contact_verified':False,'placement_checkpoint_required':False,","'cube_contact_verified':False,'placement_checkpoint_required':True,")
    gate=once(gate,"r=json.loads(Path(path).read_bytes());raw=plan.source_position();rows=plan.plan(raw)",
        "r=json.loads(Path(path).read_bytes());raw=plan.source_position();rows=plan.plan(raw)\n"
        "    checkpoint=next(i for i,(phase,_) in enumerate(rows) if phase==plan.CAPTURE_BEFORE_PHASE)\n"
        "    if checkpoint==0 or any(not np.array_equal(q,rows[checkpoint-1][1]) for _,q in rows[checkpoint:]):\n"
        "        raise RuntimeError('Placement capture contains post-placement motion; no writes')")
    (ROOT/'reverse_place_capture_gate_20261025.py').write_text(gate,encoding='utf-8')
    cmd='@echo off\nchcp 65001 >nul\ncd /d "%~dp0"\n"%~dp0.runtime\\python\\python.exe" -X utf8 -B "%~dp0leap_reverse_place_capture_20261025.py" --execute\npause\n'
    (ROOT/'leap_reverse_place_capture.cmd').write_bytes(cmd.replace('\n','\r\n').encode('utf-8'))
    from reverse_place_capture_plan_20261025 import ReversePlacePlan,SOURCE,SOURCE_SHA
    from hand_motor_mapping import SIM_TO_MOTOR_IDS
    blob=(ROOT/SOURCE).read_bytes();r=json.loads(blob);assert hashlib.sha256(blob).hexdigest()==SOURCE_SHA
    current=ROOT/'reverse_start_placement_20261025_074317_334798/fresh_off_request.json';current_blob=current.read_bytes();fresh=json.loads(current_blob)
    previous=json.loads(current.with_name('result.json').read_bytes())
    assert previous['off_start_endpoint_stable'] and previous['write_events']==[] and previous['all_torque_off_verified']
    state=fresh['off_state']
    request=dict(plan_version=ReversePlacePlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
        off_state=state,development_fixture_only=True,actual_readonly_source=str(current.relative_to(ROOT)),
        source_sha256=hashlib.sha256(current_blob).hexdigest(),reached_start_reference=SOURCE,reached_start_reference_sha256=SOURCE_SHA)
    folder=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/placement_fixture_v8';folder.mkdir(exist_ok=True)
    (folder/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
    p=ReversePlacePlan(folder/'request.json');rows=p.plan(p.source_position())
    print('PLACEMENT_RUNTIME_READY','targets',len(rows),'final_off_position',state['position'])

if __name__=='__main__':main()
