"""Standalone native screen and validator for PLACE/REMOVE experiment."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def replace(text,old,new):
    if text.count(old)!=1:raise ValueError('Expected one source block: '+old[:110])
    return text.replace(old,new,1)
def main():
    s=(ROOT/'screen_quarter_finish_empty_20261025.py').read_text(encoding='utf-8')
    s=replace(s,'from quarter_finish_empty_plan_20261025 import QuarterFinishEmptyPlan as BookGraspPlan,TURN_PHASE,END_HOLD,FINISH_PHASE,empty_basis,finish_goals',
        'from loaded_quarter_finish_plan_20261025 import LoadedQuarterFinishPlan as BookGraspPlan,TURN_PHASE,END_HOLD,FINISH_PHASE,loaded_basis,finish_goals')
    s=replace(s,"if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:\n        raise ValueError('EMPTY finish must have no PLACE checkpoint')",
        "if not plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE!=TURN_PHASE:\n        raise ValueError('Loaded finish requires a PLACE checkpoint')")
    s=replace(s,'if phase==TURN_PHASE and not inserted:inserted=True',"if phase==TURN_PHASE and not inserted:\n                dynamics.extend([('native_placement_hold',dynamics[-1][1])]*80);inserted=True")
    s=replace(s,'accepted_for_supervised_grasp_approach=False','accepted_for_supervised_grasp_approach=accepted')
    s=replace(s,'empty_hand_only=True','empty_hand_only=False')
    s=replace(s,'full_empty_return_and_finish_hand_route_checked=True','full_empty_return_and_loaded_finish_hand_route_checked=True')
    s=replace(s,'experimental_loaded_trial=False,empty_motion_basis=empty_basis()','experimental_loaded_trial=True,loaded_motion_basis=loaded_basis()')
    s=replace(s,'cube_contact_verified=False,placement_checkpoint_required=False','cube_contact_verified=False,placement_checkpoint_required=True')
    s=replace(s,'native_placement_hold_samples=0','native_placement_hold_samples=80')
    s=s.replace('Complete EMPTY return, reference and root finish. Cube absent. No physical rotation claim.','Complete EMPTY return and the EMPTY-verified reference plus root finish. Explicit PLACE/REMOVE required. Cube disabled in this hand-path screen. No physical rotation claim.')
    (ROOT/'screen_loaded_quarter_finish_20261025.py').write_text(s,encoding='utf-8')
    g=(ROOT/'quarter_finish_empty_gate_20261025.py').read_text(encoding='utf-8')
    g=replace(g,'from quarter_finish_empty_plan_20261025 import TURN_PHASE,FINISH_PHASE,END_HOLD,empty_basis,finish_goals',
        'from loaded_quarter_finish_plan_20261025 import TURN_PHASE,FINISH_PHASE,END_HOLD,loaded_basis,finish_goals')
    g=replace(g,"checker=ROOT/'screen_quarter_finish_empty_20261025.py'","checker=ROOT/'screen_loaded_quarter_finish_20261025.py'")
    g=replace(g,"if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:\n        raise RuntimeError('EMPTY finish must have no PLACE checkpoint; no writes')",
        "if not plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE!=TURN_PHASE:\n        raise RuntimeError('Loaded finish requires explicit PLACE/REMOVE checkpoints; no writes')")
    g=replace(g,"'accepted_for_supervised_grasp_approach':False","'accepted_for_supervised_grasp_approach':True")
    g=replace(g,"'empty_hand_only':True","'empty_hand_only':False")
    g=replace(g,"'full_empty_return_and_finish_hand_route_checked':True","'full_empty_return_and_loaded_finish_hand_route_checked':True")
    g=replace(g,"'experimental_loaded_trial':False","'experimental_loaded_trial':True")
    g=replace(g,"'placement_checkpoint_required':False","'placement_checkpoint_required':True")
    g=replace(g,"r.get('native_placement_hold_samples')!=0 or r.get('empty_motion_basis')!=empty_basis()","r.get('native_placement_hold_samples')!=80 or r.get('loaded_motion_basis')!=loaded_basis()")
    (ROOT/'loaded_quarter_finish_gate_20261025.py').write_text(g,encoding='utf-8');print('Built standalone PLACE/REMOVE screen and independent validator')
if __name__=='__main__':main()
