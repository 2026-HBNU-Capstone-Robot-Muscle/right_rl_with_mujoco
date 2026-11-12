"""Build isolated loaded copies; preserve the accepted EMPTY implementation."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def rep(s,a,b,n=1):
    if s.count(a)!=n:raise ValueError('Unexpected block: '+a[:100])
    return s.replace(a,b,n)
def main():
    sampler=(ROOT/'fresh_voltage_sample_20261026.py').read_text(encoding='utf-8')
    sampler=rep(sampler,'EMPTY-only sampler:','Supervised loaded sampler:')
    sampler=rep(sampler,"version='EMPTY_immediate_second_healthy_voltage_read_v1_20261026'","version='LOADED_immediate_second_healthy_voltage_read_v1_20261026'")
    sampler=rep(sampler,'empty_only=True','empty_only=False')
    sampler=rep(sampler,"if not getattr(plan,'IMMEDIATE_HEALTHY_VOLTAGE_RECHECK',False) or getattr(plan,'CAPTURE_PLACEMENT',False):",
        "if not getattr(plan,'SUPERVISED_LOADED_THUMB_MODE',False) or not getattr(plan,'CAPTURE_PLACEMENT',False):")
    sampler=sampler.replace('Immediate healthy voltage confirmation is EMPTY-only','Immediate healthy voltage confirmation requires reviewed LOADED thumb mode')
    (ROOT/'loaded_thumb_voltage_sample_20261026.py').write_text(sampler,encoding='utf-8')
    runtime=(ROOT/'camera_gap_current_runtime_20261026.py').read_text(encoding='utf-8')
    runtime=rep(runtime,"if getattr(plan_type,'CAMERA_GAP_HOLD',False):\n            if capture_mode or not getattr(camera,'EMPTY_ONLY',False):raise RuntimeError('Camera gap hold is EMPTY-only')\n            camera.bind_motion_hold();bus.camera_motion_guard=camera.motion_guard",
        "if getattr(plan_type,'CAMERA_GAP_HOLD_EARLY_EMPTY',False):\n            if not capture_mode or not getattr(plan_type,'SUPERVISED_LOADED_THUMB_MODE',False):raise RuntimeError('Invalid loaded mode scope')\n            from loaded_thumb_rotation_camera_20261026 import BeforePlacementGuard\n            camera.bind_motion_hold();bus.camera_motion_guard=BeforePlacementGuard(camera.motion_guard,report)")
    runtime=runtime.replace('Use leap_thumb_clear_root_empty_20261026.py','Use leap_loaded_thumb_rotation_20261026.py')
    (ROOT/'loaded_thumb_rotation_runtime_20261026.py').write_text(runtime,encoding='utf-8')
    s=(ROOT/'screen_thumb_clear_root_empty_20261026.py').read_text(encoding='utf-8')
    s=rep(s,'from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan as BookGraspPlan,','from loaded_thumb_rotation_plan_20261026 import LoadedThumbRotationPlan as BookGraspPlan,')
    s=rep(s,'from empty_camera_gap_hold_20261026 import metadata as gap_metadata','from loaded_thumb_rotation_camera_20261026 import metadata as gap_metadata')
    s=rep(s,'from fresh_voltage_sample_20261026 import metadata as voltage_metadata','from loaded_thumb_voltage_sample_20261026 import metadata as voltage_metadata')
    s=rep(s,'from confirmed_thumb_rest_policy_20261026 import metadata as interrupted_contact_policy,','from loaded_thumb_rest_policy_20261026 import metadata as interrupted_contact_policy,')
    s=rep(s,'from return_corner_resolution_20261026 import classify as classify_return_corner,metadata as return_corner_metadata',
        'from loaded_thumb_return_model_resolution_20261026 import classify as classify_return_corner,metadata as return_corner_metadata')
    s=rep(s,"if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:\n        raise ValueError('EMPTY candidate must not permit a loaded checkpoint')",
        "if not plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE!=TURN_PHASE:\n        raise ValueError('Loaded candidate needs the explicit PLACE checkpoint')")
    s=rep(s,'inserted=True  # No placement hold exists in the EMPTY execution.',"dynamics.append(('native_placement_hold',dynamics[-1][1],4.));inserted=True")
    for a,b in (('native_placement_hold_seconds=0.','native_placement_hold_seconds=4.'),('native_placement_hold_samples=0','native_placement_hold_samples=80'),
        ('empty_hand_only=True','empty_hand_only=False'),('experimental_loaded_trial=False','experimental_loaded_trial=True'),
        ('placement_checkpoint_required=False','placement_checkpoint_required=True')):s=rep(s,a,b)
    (ROOT/'screen_loaded_thumb_rotation_20261026.py').write_text(s,encoding='utf-8')
    g=(ROOT/'thumb_clear_root_empty_gate_20261026.py').read_text(encoding='utf-8')
    g=rep(g,"checker=ROOT/'screen_thumb_clear_root_empty_20261026.py'","checker=ROOT/'screen_loaded_thumb_rotation_20261026.py'")
    g=rep(g,'from thumb_clear_root_empty_plan_20261026 import TURN_PHASE,','from loaded_thumb_rotation_plan_20261026 import TURN_PHASE,')
    g=rep(g,'from empty_camera_gap_hold_20261026 import metadata as gap_metadata','from loaded_thumb_rotation_camera_20261026 import metadata as gap_metadata')
    g=rep(g,'from fresh_voltage_sample_20261026 import metadata as voltage_metadata','from loaded_thumb_voltage_sample_20261026 import metadata as voltage_metadata')
    g=rep(g,'from confirmed_thumb_rest_policy_20261026 import metadata as interrupted_contact_policy,','from loaded_thumb_rest_policy_20261026 import metadata as interrupted_contact_policy,')
    g=rep(g,'from return_corner_resolution_20261026 import metadata as return_corner_metadata,PAIR as RETURN_CORNER_PAIR,NUMERICAL_ZERO_MM as RETURN_ZERO_MM',
        'from loaded_thumb_return_model_resolution_20261026 import metadata as return_corner_metadata,accepted_discard')
    g=rep(g,'any(p!=RETURN_CORNER_PAIR or not 0<=d<=RETURN_ZERO_MM for p,d in v.items())',
        'any(not accepted_discard(p,d) for p,d in v.items())')
    g=rep(g,"getattr(plan,'CAMERA_GAP_HOLD',False)","getattr(plan,'CAMERA_GAP_HOLD_EARLY_EMPTY',False)")
    g=rep(g,"if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:\n        raise RuntimeError('EMPTY proposal must not permit loaded motion; no writes')",
        "if not plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE!=TURN_PHASE:\n        raise RuntimeError('Loaded proposal needs explicit PLACE/REMOVE; no writes')")
    for a,b in (("r.get('native_placement_hold_seconds')!=0.","r.get('native_placement_hold_seconds')!=4."),
        ("'empty_hand_only':True","'empty_hand_only':False"),("'experimental_loaded_trial':False","'experimental_loaded_trial':True"),
        ("'placement_checkpoint_required':False","'placement_checkpoint_required':True"),
        ("r.get('native_placement_hold_samples')!=0","r.get('native_placement_hold_samples')!=80")):g=rep(g,a,b)
    (ROOT/'loaded_thumb_rotation_gate_20261026.py').write_text(g,encoding='utf-8')
    print('Built isolated loaded sampler/runtime/checker/gate; EMPTY sources preserved.')
if __name__=='__main__':main()
