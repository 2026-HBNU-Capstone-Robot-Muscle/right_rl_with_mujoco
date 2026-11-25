"""Build inspectable standalone EMPTY screen/gate from the reviewed hand screen."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def replace(text,old,new):
    if text.count(old)!=1:raise ValueError('Expected one source block: '+old[:110])
    return text.replace(old,new,1)
def main():
    s=(ROOT/'screen_support_photo_path_20261025.py').read_text(encoding='utf-8')
    s=replace(s,'from support_photo_path_plan_20261025 import SupportPhotoPathPlan as BookGraspPlan',
        'from quarter_finish_empty_plan_20261025 import QuarterFinishEmptyPlan as BookGraspPlan,TURN_PHASE,END_HOLD,FINISH_PHASE,empty_basis,finish_goals')
    s=replace(s,"if not plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE!=TURN_PHASE:\n        raise ValueError('Loaded trial requires explicit placement checkpoint')",
        "if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:\n        raise ValueError('EMPTY finish must have no PLACE checkpoint')")
    s=replace(s,"if phase==TURN_PHASE and not inserted:\n                dynamics.extend([('native_placement_hold',dynamics[-1][1])]*80);inserted=True",'if phase==TURN_PHASE and not inserted:inserted=True')
    s=replace(s,'accepted_for_supervised_empty_preparation=accepted,accepted_for_supervised_grasp_approach=accepted','accepted_for_supervised_empty_preparation=accepted,accepted_for_supervised_grasp_approach=False')
    s=replace(s,'empty_hand_only=False','empty_hand_only=True')
    s=replace(s,'full_empty_return_and_loaded_reference_hand_route_checked=True','full_empty_return_and_finish_hand_route_checked=True')
    s=replace(s,'experimental_loaded_trial=True,experimental_model_review=validate_experiment_basis()','experimental_loaded_trial=False,empty_motion_basis=empty_basis()')
    s=replace(s,'cube_contact_verified=False,placement_checkpoint_required=True','cube_contact_verified=False,placement_checkpoint_required=False')
    s=replace(s,'native_placement_hold_samples=80','native_placement_hold_samples=0')
    s=replace(s,"note='Scoped observed passive", "empty_root_finish_goal_count=sum(p==FINISH_PHASE for p,q in candidate_rows),\n        note='EMPTY reference plus bounded root finish. Scoped observed passive")
    s=s.replace('Complete EMPTY return and proposed loaded reference hand route. Separate conditional Cube diagnostic required; actual loaded response unverified.','Complete EMPTY return, reference and root finish. Cube absent. No physical rotation claim.')
    (ROOT/'screen_quarter_finish_empty_20261025.py').write_text(s,encoding='utf-8')
    g=(ROOT/'loaded_quarter_trial_gate_20261025.py').read_text(encoding='utf-8')
    g=replace(g,"REVISION='CAD_tip_contact_diagnostic_v1_20261025'","from quarter_finish_empty_plan_20261025 import TURN_PHASE,FINISH_PHASE,END_HOLD,empty_basis,finish_goals\nREVISION='CAD_tip_contact_diagnostic_v1_20261025'")
    g=replace(g,"checker=ROOT/'screen_loaded_quarter_trial_20261025.py'","checker=ROOT/'screen_quarter_finish_empty_20261025.py'")
    g=replace(g,"if not plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE!=TURN_PHASE:\n        raise RuntimeError('Loaded experiment requires explicit PLACE/REMOVE checkpoints; no writes')",
        "if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:\n        raise RuntimeError('EMPTY finish must have no PLACE checkpoint; no writes')\n    finish=np.array([q for p,q in rows if p==FINISH_PHASE])\n    holds=np.array([q for p,q in rows if p==END_HOLD])\n    if not np.array_equal(finish,finish_goals()[1:]) or holds.shape!=(60,16) or not np.all(holds==finish_goals()[-1]) or r.get('empty_root_finish_goal_count')!=64:\n        raise RuntimeError('EMPTY root finish/hold mismatch; no writes')")
    g=replace(g,"'accepted_for_supervised_grasp_approach':True","'accepted_for_supervised_grasp_approach':False")
    g=replace(g,"'empty_hand_only':False","'empty_hand_only':True")
    g=replace(g,"'full_empty_return_and_loaded_reference_hand_route_checked':True","'full_empty_return_and_finish_hand_route_checked':True")
    g=replace(g,"'experimental_loaded_trial':True","'experimental_loaded_trial':False")
    g=replace(g,"'placement_checkpoint_required':True","'placement_checkpoint_required':False")
    g=replace(g,"r.get('native_placement_hold_samples')!=80 or r.get('experimental_model_review')!=validate_experiment_basis()","r.get('native_placement_hold_samples')!=0 or r.get('empty_motion_basis')!=empty_basis()")
    tail="""    from support_photo_contact_resolution_20261025 import metadata,PAIRS,NUMERICAL_ZERO_MM
    if r.get('photo_initial_thumb_contact_resolution')!=metadata() or r.get('initial_resolution_forces_preserved') is not True:
        raise RuntimeError('Initial numerical resolution metadata mismatch; no writes')
    discarded=r.get('initial_resolution_discarded_maximums_by_front')
    if not isinstance(discarded,list) or len(discarded)!=3 or any(not isinstance(v,dict) or not set(v).issubset(PAIRS) or any(not isinstance(x,(int,float)) or not 0<=x<=NUMERICAL_ZERO_MM for x in v.values()) for v in discarded):
        raise RuntimeError('Initial numerical resolution evidence mismatch; no writes')
    return r"""
    g=replace(g,'\n    return r\n','\n'+tail+'\n')
    (ROOT/'quarter_finish_empty_gate_20261025.py').write_text(g,encoding='utf-8')
    print('Built standalone EMPTY screen and independent gate; parent files untouched')
if __name__=='__main__':main()
