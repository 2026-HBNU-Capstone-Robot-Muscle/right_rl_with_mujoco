"""Independent gate for initial book-support separation, not free-space proof."""
import json
from pathlib import Path
import numpy as np
from book_empty_preparation_plan import (FRONTS,LIFT,MAX_INITIAL_SUPPORT_MM,ALLOWED_SUPPORT_PAIRS,
    SUPPORT_ESCAPE_STATIC_GOALS,SUPPORT_ESCAPE_DYNAMIC_SECONDS,offsets,code_hashes)
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from thumb_live_start_gate import run_screen as run_native_screen

ROOT=Path(__file__).resolve().parent


def run_screen(request,output,**kwargs):
    return run_native_screen(request,output,checker=ROOT/'screen_book_empty_preparation.py',
        failure_details=screen_failure_details,**kwargs)


def screen_failure_details(path):
    """Describe actual book-screen failures instead of generic old fields."""
    try:
        r=json.loads(Path(path).read_bytes());notes=[]
        if r.get('code_unchanged') is False:notes.append('request code or geometry changed during the screen')
        static=r.get('static_screens',[]);dynamic=r.get('dynamic_sensitivity',[])
        outside=sorted({pair for s in static+dynamic for pair in s.get('initial_support_contacts_mm',{})
                        if pair not in ALLOWED_SUPPORT_PAIRS})
        if outside:notes.append('initial support contact is outside the lifted fingers: '+', '.join(outside[:2]))
        depth=max((v for s in static+dynamic for v in s.get('initial_support_contacts_mm',{}).values()),default=0.)
        if depth>MAX_INITIAL_SUPPORT_MM:notes.append(f'initial support model depth {depth:.3f}mm exceeds {MAX_INITIAL_SUPPORT_MM:g}mm')
        if any(s.get('support_escape_max_unapproved_overlap_mm',0)>1e-6 for s in static+dynamic):
            notes.append('a new/deeper support contact appeared, or separation missed its deadline')
        if any(s.get('support_escape_complete') is False for s in static+dynamic):
            notes.append('initial support contact did not separate before its deadline')
        if any(s.get('max_self_overlap_mm',0)>1e-6 for s in static+dynamic):notes.append('finger self-contact did not clear')
        if any(s.get('native_range_violation_rad',0)>1e-6 for s in static+dynamic):notes.append('native joint range check failed')
        if any(s.get('max_tracking_rad',0)>.12 or s.get('finite') is False for s in dynamic):
            notes.append('simulated tracking/finite-state check failed')
        return '; '.join(notes[:3]) or 'full book-support separation checks did not pass'
    except (OSError,ValueError,KeyError,TypeError):return 'no readable completed book-support screen was saved'


def validate_proof(plan,path):
    r=json.loads(Path(path).read_bytes());raw=plan.source_position();rows=plan.plan(raw)
    if (r.get('accepted_for_supervised_empty_preparation') is not True
            or r.get('free_space_certificate') is not False
            or r.get('physical_calibration_verified') is not False
            or r.get('native_ranges_unchanged') is not True
            or r.get('code_unchanged') is not True
            or r.get('plan_version')!=plan.PLAN_VERSION or r.get('source_sha256')!=plan.SOURCE_SHA
            or r.get('plan_sha256')!=plan.plan_digest(raw) or r.get('start_ticks')!=raw.tolist()
            or r.get('last_goal_ticks')!=rows[-1][1].tolist() or r.get('mapping')!=list(SIM_TO_MOTOR_IDS)
            or r.get('code_hashes')!=code_hashes() or r.get('hardware_commands_sent')!=0):
        raise RuntimeError('Book preparation proof does not match current request/path/geometry/code; no writes')
    static=r.get('static_screens',[]);dynamic=r.get('dynamic_sensitivity',[])
    if len(static)!=67*len(FRONTS) or len(dynamic)!=3*len(FRONTS):
        raise RuntimeError('Book preparation sensitivity cases incomplete; no writes')
    lift_count=sum(p==LIFT for p,_ in rows)
    def inspect(row,is_dynamic):
        pairs=row.get('initial_support_contacts_mm')
        if not isinstance(pairs,dict) or not set(pairs).issubset(ALLOWED_SUPPORT_PAIRS):return False
        depths=np.array(list(pairs.values()),dtype=float)
        if not np.isfinite(depths).all() or np.any(depths<0) or np.any(depths>MAX_INITIAL_SUPPORT_MM):return False
        keys=('max_self_overlap_mm','max_support_after_lift_mm','support_escape_max_unapproved_overlap_mm',
              'support_escape_max_step_increase_mm','native_range_violation_rad','support_escape_complete_at',
              'support_escape_deadline')
        if any(row.get(k) is None for k in keys):return False
        a=np.array([row[k] for k in keys],dtype=float)
        if not np.isfinite(a).all() or np.any(a<0):return False
        deadline=(min(.5+lift_count*.05,SUPPORT_ESCAPE_DYNAMIC_SECONDS) if is_dynamic
                  else min(lift_count,SUPPORT_ESCAPE_STATIC_GOALS))
        if (a[0]>1e-6 or a[1]>.15 or a[2]>1e-6 or a[4]>1e-6
                or a[6]!=deadline or a[5]>deadline or row.get('support_escape_complete') is not True):return False
        if is_dynamic:
            tracking=row.get('max_tracking_rad',float('nan'))
            return row.get('finite') is True and np.isfinite(tracking) and 0<=tracking<=.12
        step=row.get('max_goal_step_ticks',float('nan'))
        return (a[3]<=1e-6 and np.isfinite(step) and step==round(step) and 0<=step<=2
                and a[5]==round(a[5]))
    expected_offsets=sorted(tuple(v.tolist()) for v in offsets())
    for front in FRONTS:
        sr=[s for s in static if s.get('book_front_mm')==front]
        dr=[s for s in dynamic if s.get('book_front_mm')==front]
        actual=[]
        for row in sr:
            off=np.asarray(row.get('offset_ticks'))
            if off.shape!=(16,) or not np.isfinite(off).all() or np.any(off!=np.rint(off)):
                raise RuntimeError('Invalid book preparation neighborhood; no writes')
            actual.append(tuple(off.tolist()))
        if (sorted(actual)!=expected_offsets or sorted(s.get('gain_scale') for s in dr)!=[.5,1.,1.5]
                or not all(inspect(s,False) for s in sr) or not all(inspect(s,True) for s in dr)):
            raise RuntimeError('Book contact separation, native limits, or dynamics rejected; no writes')
    return r
