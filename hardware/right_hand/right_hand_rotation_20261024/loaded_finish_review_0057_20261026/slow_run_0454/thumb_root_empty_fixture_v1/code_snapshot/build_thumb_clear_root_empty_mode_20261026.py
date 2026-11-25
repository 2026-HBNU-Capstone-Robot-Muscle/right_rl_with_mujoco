"""Create isolated EMPTY checker/validator, preserving all older modes."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def rep(s,a,b,n=1):
    if s.count(a)!=n:raise ValueError('Unexpected source block: '+a[:100])
    return s.replace(a,b,n)
def main():
    s=(ROOT/'screen_slow_forward_20261026.py').read_text(encoding='utf-8')
    s=rep(s,'from slow_forward_plan_20261026 import SlowForwardPlan as BookGraspPlan,TURN_PHASE,END_HOLD,FINISH_PHASE,loaded_basis,finish_goals',
          'from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan as BookGraspPlan,TURN_PHASE,END_HOLD,FINISH_PHASE,loaded_basis,finish_goals,proposal_metadata')
    s=rep(s,"if not plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE!=TURN_PHASE:\n        raise ValueError('Loaded finish requires a PLACE checkpoint')",
          "if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:\n        raise ValueError('EMPTY candidate must not permit a loaded checkpoint')")
    s=rep(s,"dynamics.append(('native_placement_hold',dynamics[-1][1],4.));inserted=True","inserted=True  # No placement hold exists in the EMPTY execution.")
    s=rep(s,'native_placement_hold_seconds=4.','native_placement_hold_seconds=0.')
    s=rep(s,'native_placement_hold_samples=80','native_placement_hold_samples=0')
    s=rep(s,'empty_hand_only=False','empty_hand_only=True')
    s=rep(s,'experimental_loaded_trial=True','experimental_loaded_trial=False')
    s=rep(s,'placement_checkpoint_required=True','placement_checkpoint_required=False')
    s=rep(s,'full_empty_return_and_loaded_finish_hand_route_checked=True','full_empty_return_and_new_thumb_root_route_checked=True')
    s=rep(s,'r=dict(slow_thumb_seam_resolution=', 'r=dict(thumb_root_proposal=proposal_metadata(),slow_thumb_seam_resolution=')
    s=rep(s,"note='EMPTY reference plus bounded root finish. Scoped observed passive ID4 recovery and 0.1mm ring-tip/palm-box classification tolerance; forces preserved. Full hand route only. Complete EMPTY return and the EMPTY-verified reference plus root finish. Explicit PLACE/REMOVE required. Cube disabled in this hand-path screen. No physical rotation claim. Explicit start and final holds; no live policy. Full CAD fingertip and middle/ring housing checks.'",
          "note='EMPTY recovery, known reference, new thumb retreat, new index/middle root lift and holds. Cube disabled for the entire screen. No loaded permission or physical rotation claim. Existing contact classifications and all forces preserved.'")
    (ROOT/'screen_thumb_clear_root_empty_20261026.py').write_text(s,encoding='utf-8')
    g=(ROOT/'slow_forward_gate_20261026.py').read_text(encoding='utf-8')
    g=rep(g,"checker=ROOT/'screen_slow_forward_20261026.py'","checker=ROOT/'screen_thumb_clear_root_empty_20261026.py'")
    g=rep(g,'from slow_forward_plan_20261026 import TURN_PHASE,FINISH_PHASE,END_HOLD,loaded_basis,finish_goals',
          'from thumb_clear_root_empty_plan_20261026 import TURN_PHASE,FINISH_PHASE,END_HOLD,loaded_basis,finish_goals,THUMB_CLEAR,ROOT_UP,tail_goals,final_goal,proposal_metadata')
    g=rep(g,"r.get('native_placement_hold_seconds')!=4.","r.get('native_placement_hold_seconds')!=0.")
    g=rep(g,"if not plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE!=TURN_PHASE:\n        raise RuntimeError('Loaded finish requires explicit PLACE/REMOVE checkpoints; no writes')",
          "if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:\n        raise RuntimeError('EMPTY proposal must not permit loaded motion; no writes')\n    expected=tail_goals();actual=[(p,q) for p,q in rows if p in (THUMB_CLEAR,ROOT_UP)]\n    if r.get('thumb_root_proposal')!=proposal_metadata() or [p for p,q in actual]!=[p for p,q in expected] or not np.array_equal([q for p,q in actual],[q for p,q in expected]):\n        raise RuntimeError('New thumb/root goal sequence mismatch; no writes')")
    g=rep(g,'not np.all(holds==finish_goals()[-1])','not np.all(holds==final_goal())')
    g=rep(g,"'empty_hand_only':False","'empty_hand_only':True")
    g=rep(g,"'experimental_loaded_trial':True","'experimental_loaded_trial':False")
    g=rep(g,"'placement_checkpoint_required':True","'placement_checkpoint_required':False")
    g=rep(g,"'full_empty_return_and_loaded_finish_hand_route_checked':True","'full_empty_return_and_new_thumb_root_route_checked':True")
    g=rep(g,"r.get('native_placement_hold_samples')!=80","r.get('native_placement_hold_samples')!=0")
    (ROOT/'thumb_clear_root_empty_gate_20261026.py').write_text(g,encoding='utf-8')
    print('Created isolated EMPTY proposal checker and independent validator; parent files unchanged.')
if __name__=='__main__':main()
