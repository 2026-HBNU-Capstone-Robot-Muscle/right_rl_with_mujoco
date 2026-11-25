"""Fork a separate EMPTY proof; leave the completed placement protocol intact."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def once(s,a,b):
    if s.count(a)!=1:raise ValueError('Ambiguous template: '+a[:80])
    return s.replace(a,b)

def main():
    screen=(ROOT/'screen_reverse_place_capture_20261025.py').read_text(encoding='utf-8')
    screen=once(screen,'LOADED_PHASE=END_HOLD',
        'LOADED_PHASE=END_HOLD\nfrom reverse_quarter_path_empty_plan_20261025 import ReverseQuarterEmptyPlan as BookGraspPlan,extra_hashes,START_HOLD,TURN_PHASE,motion_goals')
    a="checkpoint=next(i for i,(phase,_) in enumerate(candidate_rows) if phase==plan.CAPTURE_BEFORE_PHASE)\n    if checkpoint==0 or any(not np.array_equal(q,candidate_rows[checkpoint-1][1]) for _,q in candidate_rows[checkpoint:]):\n        raise ValueError('Placement observation contains moving joint goals')"
    b="if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:\n        raise ValueError('Whole EMPTY path must not include placement')\n    if sum(p==TURN_PHASE for p,q in candidate_rows)!=len(motion_goals())-1:\n        raise ValueError('Incomplete EMPTY reference motion')"
    screen=once(screen,a,b)
    screen=once(screen,"            if phase==LOADED_PHASE and not inserted:\n                dynamics.extend([('placement_hold',dynamics[-1][1])]*40);inserted=True\n",'')
    screen=once(screen,'empty_hand_only=False,placement_capture_only=True,joint_goals_fixed_after_placement=True,',
        'empty_hand_only=True,placement_capture_only=False,joint_goals_fixed_after_placement=False,full_quarter_reference_path=True,')
    screen=once(screen,'cube_contact_verified=False,placement_checkpoint_required=True,','cube_contact_verified=False,placement_checkpoint_required=False,')
    screen=once(screen,'native_placement_hold_samples=40','native_placement_hold_samples=0')
    screen=screen.replace('Photo-only placement checkpoint. All joint goals remain fixed after placement. No rotation or live policy.',
        'Whole EMPTY quarter-turn reference once. No placement or loaded Cube turn. Explicit start and final holds; no live policy.')
    (ROOT/'screen_reverse_quarter_path_empty_20261025.py').write_text(screen,encoding='utf-8')
    gate=(ROOT/'reverse_place_capture_gate_20261025.py').read_text(encoding='utf-8')
    gate=once(gate,'from reverse_place_capture_plan_20261025 import extra_hashes',
        'from reverse_quarter_path_empty_plan_20261025 import extra_hashes,START_HOLD,TURN_PHASE,motion_goals')
    a="checkpoint=next(i for i,(phase,_) in enumerate(rows) if phase==plan.CAPTURE_BEFORE_PHASE)\n    if checkpoint==0 or any(not np.array_equal(q,rows[checkpoint-1][1]) for _,q in rows[checkpoint:]):\n        raise RuntimeError('Placement capture contains post-placement motion; no writes')"
    b="if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:\n        raise RuntimeError('Unexpected placement in EMPTY path; no writes')\n    motion=np.array([q for p,q in rows if p==TURN_PHASE])\n    if not np.array_equal(motion,motion_goals()[1:]) or sum(p==START_HOLD for p,q in rows)!=40:\n        raise RuntimeError('Whole EMPTY motion/hold mismatch; no writes')"
    gate=once(gate,a,b)
    gate=once(gate,"'empty_hand_only':False,'placement_capture_only':True,'joint_goals_fixed_after_placement':True,",
        "'empty_hand_only':True,'placement_capture_only':False,'joint_goals_fixed_after_placement':False,'full_quarter_reference_path':True,")
    gate=once(gate,"'cube_contact_verified':False,'placement_checkpoint_required':True,","'cube_contact_verified':False,'placement_checkpoint_required':False,")
    gate=once(gate,"r.get('native_placement_hold_samples')!=40","r.get('native_placement_hold_samples')!=0")
    gate=once(gate,"checker=ROOT/'screen_reverse_place_capture_20261025.py'","checker=ROOT/'screen_reverse_quarter_path_empty_20261025.py'")
    (ROOT/'reverse_quarter_path_empty_gate_20261025.py').write_text(gate,encoding='utf-8')
    cmd='@echo off\nchcp 65001 >nul\ncd /d "%~dp0"\n"%~dp0.runtime\\python\\python.exe" -X utf8 -B "%~dp0leap_reverse_quarter_path_empty_20261025.py" --execute\npause\n'
    (ROOT/'leap_reverse_quarter_path_empty.cmd').write_bytes(cmd.replace('\n','\r\n').encode('utf-8'))
    from reverse_quarter_path_empty_plan_20261025 import ReverseQuarterEmptyPlan
    from hand_motor_mapping import SIM_TO_MOTOR_IDS
    source=json.loads((ROOT/'right_hand_rotation_20261024/reverse_quarter_path_empty/actual_off_source.json').read_bytes())
    request=dict(plan_version=ReverseQuarterEmptyPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=source['measured_off_state'],
        development_fixture_only=True,recorded_source=source['source'],recorded_source_sha256=source['source_sha256'])
    folder=ROOT/'right_hand_rotation_20261024/reverse_quarter_path_empty/fixture_v1';folder.mkdir(exist_ok=True)
    (folder/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
    plan=ReverseQuarterEmptyPlan(folder/'request.json');rows=plan.plan(plan.source_position())
    print('EMPTY_REFERENCE_GOALS',len(rows),'nominal_seconds_including_final_hold',(len(rows)+20)*.05)

if __name__=='__main__':main()
