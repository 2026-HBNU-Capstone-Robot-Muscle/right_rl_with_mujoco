"""Bind the narrow stable-placement change to measured geometry and stress data."""
import hashlib,json,math
from pathlib import Path
import numpy as np
from book_empty_preparation_plan import FRONTS,offsets
from horizontal_placement_settle_20261027 import POLICY
ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'right_hand_rotation_20261027/open_settle_221539'
GEOMETRY=FOLDER/'placement_bound_geometry.json'
WEAK=FOLDER/'weaker_root_matrix/review.json'
WEAK_PATH=WEAK.parent/'loaded_path.npz'
LOADED=ROOT/'right_hand_rotation_20261027/yaw90_clearance_t8_early_root16/full_loaded_27/loaded_path.npz'
SOURCE=ROOT/'horizontal_quarter_trial_20261027_221539_291945/result.json'
FILES=tuple(v.relative_to(ROOT).as_posix() for v in (GEOMETRY,WEAK,WEAK_PATH,SOURCE))

def bounded(row,bounds):
    return all(isinstance(row.get(k),(int,float)) and not isinstance(row[k],bool)
        and math.isfinite(row[k]) and 0<=row[k]<=v for k,v in bounds.items())

def geometry_review(data):
    source=json.loads(SOURCE.read_bytes());samples=[v for v in source['samples'] if v['phase']=='placement_checkpoint']
    extra=[]
    for sign in (-1,1):
        a=[0]*16;a[11]=sign*5;extra.append(a)
    expected={(front,tuple(delta)) for front in FRONTS for delta in list(offsets())+extra}
    cases=data.get('cases',[])
    if (data.get('all_passed') is not True or len(cases)!=207 or len(samples)!=10
        or data.get('observed_actual_ticks')!=samples[-1]['actual']
        or data.get('open_goal_ticks')!=samples[-1]['goal']
        or data.get('source')!=SOURCE.relative_to(ROOT).as_posix()
        or data.get('id12_maximum_error_ticks')!=72
        or data.get('geometry_and_ranges_changed') is not False or data.get('motor_writes')!=0
        or {(v.get('book_front_mm'),tuple(v.get('offset_ticks',[]))) for v in cases}!=expected):
        raise RuntimeError('Stable placement lacks measured boundary geometry')
    if any(v.get('passed') is not True or v.get('native') is not True
           or not bounded(v,{'max_self_mm':1e-6,'max_support_mm':.15}) for v in cases):
        raise RuntimeError('Stable placement boundary geometry rejected')
    return dict(cases=207,id12_maximum_error_ticks=72,measured_source=SOURCE.relative_to(ROOT).as_posix())

def weak_review(data):
    cases=data.get('cases',[]);expected={(f,g,mu) for f in FRONTS for g in (.5,1.,1.5) for mu in (.2,.3,.5)}
    if (len(cases)!=27 or {(v.get('book_front_mm'),v.get('gain_scale'),v.get('friction')) for v in cases}!=expected
        or data.get('response_multiplier_by_root')!={'8':1.3,'12':.5,'16':.7}
        or data.get('required_response_scale')!=.5 or data.get('required_cases')!=9
        or data.get('required_cases_passed') is not True or data.get('loaded_arrays_unchanged') is not True
        or data.get('matching_is_stress_hypothesis_not_verified_calibration') is not True
        or data.get('finite_stiffness_contact_forces_preserved') is not True
        or data.get('hardware_execution_allowed') is not False or data.get('physical_yaw_verified') is not False):
        raise RuntimeError('Missing weaker-response loaded stress evidence')
    required=[v for v in cases if v['gain_scale']==.5]
    for row in required:
        if (row.get('passed') is not True or row.get('stop') is not None
            or row.get('last_sample')!=2742 or row.get('last_phase')!='remove_hold'
            or row.get('external_cube_contact') is not False or row.get('final_cube_supported_by_hand') is not True
            or not bounded(row,{'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_mm':1,
                'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6,'center_translation_mm':80,
                'longest_no_hand_contact_seconds':.25,'yaw_before_release_deg':180})):
            raise RuntimeError('Weaker-response turn safety/retention rejected')
    return dict(required_cases=9,additional_comparison_cases=18,
        yaw_range_deg=[min(v['yaw_before_release_deg'] for v in required),max(v['yaw_before_release_deg'] for v in required)],
        calibrated_model=False,robust_90_degree_rotation_verified=False)

def review():
    geometry=geometry_review(json.loads(GEOMETRY.read_bytes()));weak=json.loads(WEAK.read_bytes());stress=weak_review(weak)
    if weak.get('loaded_path_sha256')!=hashlib.sha256(WEAK_PATH.read_bytes()).hexdigest():
        raise RuntimeError('Weaker-response stress targets changed')
    with np.load(LOADED) as original,np.load(WEAK_PATH) as additional:
        if set(original.files)!=set(additional.files) or any(not np.array_equal(original[k],additional[k]) for k in original.files):
            raise RuntimeError('Weaker-response stress changed loaded goals')
    return dict(policy=dict(POLICY),geometry=geometry,weaker_response=stress,
        evidence_sha256={v:hashlib.sha256((ROOT/v).read_bytes()).hexdigest() for v in FILES})
