"""Match fixed grasp cycle and model review before any motor settings writes."""
import json,math
from pathlib import Path
import numpy as np
from new_pair_grasp_plan_20261027 import NewPairGraspPlan,ROOT,REQUEST,PROOF,hashes,loaded_digest
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from thumb_root_off_screen_process_20261026 import run_screen as process_screen
def finite(v):return isinstance(v,(int,float)) and math.isfinite(v) and v>=0
def validate_proof(plan,path):
    r=json.loads(Path(path).read_bytes());a=plan.source_position();rows=plan.plan(a)
    if r.get('accepted_for_operator_grasp_trial') is not True or r.get('code_unchanged') is not True or r.get('physical_grasp_verified') is not False or r.get('physical_rotation_verified') is not False or r.get('hardware_commands_sent')!=0:raise RuntimeError('Grasp model review did not pass; no motion started')
    if r.get('plan_version')!=plan.PLAN_VERSION or r.get('source_sha256')!=plan.SOURCE_SHA or r.get('plan_sha256')!=plan.plan_digest(a) or r.get('loaded_cycle_sha256')!=loaded_digest(plan) or r.get('start_ticks')!=a.tolist() or r.get('last_goal_ticks')!=rows[-1][1].tolist() or r.get('mapping')!=list(SIM_TO_MOTOR_IDS) or r.get('code_hashes')!=hashes():raise RuntimeError('Grasp source/queue/code mismatch; no writes')
    expected={'goal_step_ticks':2,'sample_period_s':.075,'initial_support_bound_mm':5,'root_escape_goals':70,'root_escape_seconds':3.5,'support_after_release_mm':.15,'self_contact_mm':1e-6,'hardware_tracking_stop_rad':.18,'passive_id4_reentry_only':True,'finite_stiffness_contact_forces_preserved':True}
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Grasp bounds changed; no writes')
    statics=r.get('static_screens',[]);dynamics=r.get('dynamic_sensitivity',[]);loaded=r.get('loaded_physics',[])
    if len(statics)!=201 or len(dynamics)!=9 or len(loaded)!=27:raise RuntimeError('Missing grasp review cases; no writes')
    from book_empty_preparation_plan import offsets,FRONTS
    if {(v['book_front_mm'],tuple(v['offset_ticks'])) for v in statics}!={(f,tuple(o)) for f in FRONTS for o in offsets()} or {(v['book_front_mm'],v['gain_scale']) for v in dynamics}!={(f,g) for f in FRONTS for g in (.5,1.,1.5)} or {(v['book_front_mm'],v['gain_scale'],v['friction']) for v in loaded}!={(f,g,mu) for f in FRONTS for g in (.5,1.,1.5) for mu in (.2,.3,.5)}:raise RuntimeError('Duplicated or missing review coverage; no writes')
    for row in statics:
        if row.get('passed') is not True or any(not finite(row.get(k)) for k in ('max_self_mm','max_support_after_release_mm','initial_support_increase_mm','step_support_increase_mm')) or any(not finite(v) or v>5 for v in row.get('initial_support_mm',{}).values()) or row['max_self_mm']>1e-6 or row['max_support_after_release_mm']>.15 or row['initial_support_increase_mm']>1e-6 or row['step_support_increase_mm']>1e-6 or not isinstance(row.get('root_clear_sample'),int) or not 0<=row['root_clear_sample']<=70 or not isinstance(row.get('root_clear_goal_ticks'),list) or len(row['root_clear_goal_ticks'])!=3 or any(not finite(v) for v in row['root_clear_goal_ticks']):raise RuntimeError('Static grasp route rejected; no writes')
    for row in dynamics:
        if row.get('passed') is not True or row.get('finite') is not True or any(not finite(row.get(k)) for k in ('max_self_mm','max_support_after_release_mm','max_tracking_rad','tracking_after_release_rad','unapproved_range_excess_rad','initial_support_increase_mm','root_clear_seconds')) or row['max_self_mm']>1e-6 or row['max_support_after_release_mm']>.15 or row['max_tracking_rad']>.16 or row['tracking_after_release_rad']>.12 or row['unapproved_range_excess_rad']>1e-6 or row['initial_support_increase_mm']>1e-6 or row['root_clear_seconds']>3.575:raise RuntimeError('Dynamic grasp route rejected; no writes')
    for row in loaded:
        if row.get('passed') is not True or row.get('finite') is not True or any(not finite(row.get(k)) for k in ('max_self_mm','max_support_mm','max_cube_mm','max_tracking_rad','two_tip_hold_seconds','unapproved_range_excess_rad')) or row['max_self_mm']>1e-6 or row['max_support_mm']>.15 or row['max_cube_mm']>1 or row['max_tracking_rad']>.18 or row['unapproved_range_excess_rad']>1e-6 or row.get('external_cube_contact') is not False or row.get('final_cube_supported_by_hand') is not True:raise RuntimeError('Loaded grasp/reopening rejected; no writes')
    nominal=[v for v in loaded if v.get('book_front_mm')==-10 and v.get('gain_scale')==1 and v.get('friction')==.3]
    if len(nominal)!=1 or nominal[0]['two_tip_hold_seconds']<3:raise RuntimeError('Nominal grasp not sustained; no writes')
    return r
def reusable_loaded_review(plan):
    reference=NewPairGraspPlan(REQUEST)
    r=validate_proof(reference,PROOF)
    if loaded_digest(plan)!=r['loaded_cycle_sha256']:raise RuntimeError('Loaded cycle changed; no cache reuse')
    return r['loaded_physics']
def run_screen(request,output,**kwargs):
    candidate=NewPairGraspPlan(request);reference=NewPairGraspPlan(REQUEST)
    if np.array_equal(candidate.source_position(),reference.source_position()):
        r=validate_proof(reference,PROOF);r=dict(r);r['source_sha256']=candidate.SOURCE_SHA;r['exact_start_cached_review_used']=True;Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_proof(candidate,output)
    process_screen(request,output,checker=ROOT/'screen_new_pair_grasp_20261027.py',failure_details=lambda p:'current recovery/grasp model review failed',**kwargs);return validate_proof(candidate,output)
