"""OFF acquisition and process/proof checks for a fresh screened thumb path."""
import hashlib
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from thumb_live_start_plan import CurrentPlan,PLAN_VERSION,validate_off_state

ROOT=Path(__file__).resolve().parent
CHECKER=ROOT/'screen_thumb_live_start.py'
PLANNER=ROOT/'thumb_live_start_plan.py'


def stable_off(bus,report,*,cancel,sleep=time.sleep,now=time.monotonic,validator=validate_off_state):
    began=now();recent=[]
    report['phase']='stable_off_readings'
    for _ in range(16):
        cancel()
        if now()-began>5:break
        read_at=now();s=bus.snapshot()
        if now()-read_at>.2:raise RuntimeError('OFF read too slow; no writes')
        validator(s)
        report.setdefault('off_start_observations',[]).append({k:v.tolist() for k,v in s.items()})
        recent.append(s);recent=recent[-3:]
        if len(recent)==3 and np.all(np.ptp(np.array([v['position'] for v in recent]),axis=0)<=2):
            report['off_start_endpoint_stable']=True
            return s
        sleep(.2)
    raise RuntimeError('OFF angles still moving; current readings saved, no writes')


def write_request(folder,state,plan_version=PLAN_VERSION,validator=validate_off_state):
    validator(state)
    path=Path(folder)/'fresh_off_request.json'
    request=dict(plan_version=plan_version,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
                 origin='operator_triggered_stable_off_reads',
                 off_state={k:np.asarray(v).tolist() for k,v in state.items()})
    path.write_text(json.dumps(request,indent=2),encoding='utf-8')
    return path


def wsl_path(path):
    value=str(Path(path).resolve()).replace('\\','/')
    if len(value)<3 or value[1:3]!=':/':raise ValueError('WSL checker requires a Windows drive path')
    return '/mnt/'+value[0].lower()+value[2:]


def screen_failure_details(path):
    """Diagnostic text only; this never accepts or edits a proof."""
    try:
        r=json.loads(Path(path).read_text(encoding='utf-8'));notes=[]
        static=r.get('static_screens',[]);dynamic=r.get('dynamic_sensitivity',[])
        if any(s.get('unapproved_native_range_violation_rad',0)>1e-6 for s in static):
            notes.append('joint-range recovery did not pass')
        travel=[s.get('contact_clear_pip_travel_ticks') for s in static if 'contact_clear_pip_travel_ticks' in s]
        # Diagnose using the same active plan budget as the checker, rather
        # than retaining a separate historical literal in the error message.
        travel_limit=64
        from grasp_place_plan import GraspPlacePlan
        from cube_small_rotation_plan import SmallRotationPlan
        from supported_yaw_empty_plan import SupportedYawEmptyPlan
        for kind in (GraspPlacePlan,SmallRotationPlan,SupportedYawEmptyPlan):
            if r.get('plan_version')==kind.PLAN_VERSION:
                travel_limit=kind.MAX_CONTACT_RELEASE_TRAVEL_TICKS
        reported_limit=r.get('contact_release_travel_limit_ticks')
        if type(reported_limit) is int and 0<=reported_limit<=4096:
            travel_limit=reported_limit
        if travel and (any(v is None for v in travel) or max(v for v in travel if v is not None)>travel_limit):
            notes.append('ring contact could not clear within the short release')
        if any(s.get('self_escape_max_unapproved_overlap_mm',0)>1e-6 for s in static+dynamic):
            notes.append('existing contact increased, a new contact appeared, or its release was not permitted')
        if any(s.get('self_escape_complete') is False for s in static+dynamic):
            notes.append('contact did not separate before its deadline')
        if any(s.get('max_support_increase_mm',0)>1e-6 for s in static) or any(s.get('max_support_overlap_mm',0)>.15 for s in dynamic):
            notes.append('hand/support clearance did not pass')
        if any(s.get('max_tracking_rad',0)>.12 or s.get('finite') is False for s in dynamic):
            notes.append('simulated motor tracking did not pass')
        return '; '.join(notes[:3]) or 'the saved geometry/dynamics checks did not pass'
    except (OSError,ValueError,TypeError,KeyError):
        return 'checker did not produce a readable diagnostic result'


def run_screen(request,output,*,cancel,sleep=time.sleep,now=time.monotonic,popen=subprocess.Popen,checker=CHECKER,
               failure_details=screen_failure_details):
    request,output=Path(request),Path(output)
    if output.exists():raise RuntimeError('Screen output must be new; no stale proof reuse')
    command=['wsl.exe','--exec','/home/star1209042/venv/bin/python',
             wsl_path(checker),wsl_path(request),wsl_path(output)]
    log=output.with_suffix('.log');began=now();last_progress=began
    with log.open('wb') as stream:
        process=popen(command,stdout=stream,stderr=subprocess.STDOUT,
                      creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        try:
            while process.poll() is None:
                cancel()
                if now()-began>120:raise RuntimeError('OFF path screen exceeded 120s; no writes')
                if now()-last_progress>5:
                    print(f'OFF path check running ({now()-began:.0f}s); Q cancels.',flush=True)
                    last_progress=now()
                sleep(.05)
            if process.returncode!=0:
                raise RuntimeError('Current-pose path screen failed: '+failure_details(output)+
                                   '; no motion started; see '+str(log))
        finally:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
    if not output.is_file():raise RuntimeError('No current-pose proof generated; no writes')
    return output


def validate_proof(plan,path):
    r=json.loads(Path(path).read_text(encoding='utf-8'));raw=plan.source_position()
    if (r.get('accepted') is not True or r.get('native_ranges_unchanged') is not True
            or r.get('plan_version')!=plan.PLAN_VERSION
            or r.get('source_sha256')!=plan.SOURCE_SHA
            or r.get('plan_sha256')!=plan.plan_digest(raw)
            or r.get('mapping')!=list(SIM_TO_MOTOR_IDS)
            or r.get('start_ticks')!=raw.tolist()
            or r.get('last_goal_ticks')!=plan.plan(raw)[-1][1].tolist()
            or r.get('script_sha256')!=hashlib.sha256(CHECKER.read_bytes()).hexdigest()
            or r.get('planner_sha256')!=hashlib.sha256(PLANNER.read_bytes()).hexdigest()
            or r.get('extra_planner_sha256',{})!={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                            for name in getattr(plan,'EXTRA_PLANNER_FILES',())}
            or r.get('hardware_commands_sent')!=0):
        raise RuntimeError('Current-pose proof does not match this request/path/code; no writes')
    static=r.get('static_screens',[]);dynamic=r.get('dynamic_sensitivity',[])
    if len(static)!=67 or len(dynamic)!=3:raise RuntimeError('Incomplete screen; no writes')
    offsets=[]
    def self_pass(row,*,dynamic=False):
        if not hasattr(plan,'INITIAL_SELF_ESCAPE_PAIRS'):return row['max_self_overlap_mm']<=1e-6
        pairs=row.get('initial_self_contacts_mm')
        if not isinstance(pairs,dict):return False
        values=np.array(list(pairs.values()),dtype=float)
        limit=plan.INITIAL_SELF_ESCAPE_DYNAMIC_S if dynamic else plan.INITIAL_SELF_ESCAPE_GOALS
        unapproved=row.get('self_escape_max_unapproved_overlap_mm',float('nan'))
        increase=row.get('self_escape_max_step_increase_mm',float('nan'))
        completed=row.get('self_escape_complete_at')
        if completed is None:return False
        depth_pass=(max(pairs.values(),default=0)<=plan.MAX_INITIAL_SELF_ESCAPE_MM
                    if plan.MAX_INITIAL_SELF_ESCAPE_MM is not None
                    else bool(getattr(plan,'CONTACT_RELEASE_BEFORE_PHASE',None)))
        certificate_pass=True
        if getattr(plan,'CONTACT_RELEASE_BEFORE_PHASE',None) and not dynamic:
            travel=row.get('contact_clear_pip_travel_ticks',float('nan'))
            threshold=row.get('contact_clear_pip_goal_ticks',float('nan'))
            if travel is None or threshold is None:return False
            certificate_pass=(np.isfinite(travel) and travel==round(travel)
                and 0<=travel<=plan.MAX_CONTACT_RELEASE_TRAVEL_TICKS
                and np.isfinite(completed) and completed==round(completed)
                and 0<=completed<=plan.INITIAL_SELF_ESCAPE_GOALS
                and travel<=completed*plan.MAX_GOAL_STEP
                and np.isfinite(threshold) and threshold==round(threshold) and 0<=threshold<=4095
                and threshold+travel==int(raw[14])+row['offset_ticks'][14])
        return (set(pairs).issubset(plan.INITIAL_SELF_ESCAPE_PAIRS)
            and np.isfinite(values).all() and np.all(values>=0)
            and depth_pass and certificate_pass
            and np.isfinite(unapproved) and 0<=unapproved<=1e-6
            and np.isfinite(increase) and increase>=0 and (dynamic or increase<=1e-6)
            and np.isfinite(completed) and 0<=completed<=limit
            and row.get('self_escape_complete') is True
            and row['max_self_overlap_mm']<=max(pairs.values(),default=0)+1e-6)
    for row in static:
        offset=np.asarray(row['offset_ticks'])
        values=np.array([row[k] for k in ('max_self_overlap_mm','initial_support_overlap_mm',
            'max_support_increase_mm','thumb_support_overlap_mm','max_goal_step_ticks','native_range_violation_rad')])
        range_pass=values[5]<=1e-6
        if getattr(plan,'NATIVE_REENTRY_PHASE',None):
            initial=row.get('initial_native_range_excess_rad',float('nan'))
            unapproved=row.get('unapproved_native_range_violation_rad',float('nan'))
            range_pass=(np.isfinite(initial) and np.isfinite(unapproved) and 0<=initial
                and 0<=unapproved<=1e-6 and initial<=plan.MAX_START_RANGE_EXCESS_RAD+1e-6
                and values[5]<=initial+1e-6
                and row.get('native_reentry_phase')==plan.NATIVE_REENTRY_PHASE
                and row.get('native_reentry_monotonic') is True
                and row.get('native_reentry_complete') is True)
        if (offset.shape!=(16,) or np.any(abs(offset)>4) or np.any(offset!=np.rint(offset))
                or not np.isfinite(values).all() or np.any(values<0)
                or not self_pass(row) or values[1]>.15 or values[2]>1e-6
                or values[3]>1e-6 or values[4]>2 or not range_pass):
            raise RuntimeError('Static clearance/limits screen rejected; no writes')
        offsets.append(offset.tolist())
    # Ensure nominal, both full corners and all single-joint +/-4 cases exist.
    required=[[0]*16,[-4]*16,[4]*16]
    for i in range(16):
        for sign in (-1,1):
            v=[0]*16;v[i]=sign*4;required.append(v)
    if any(v not in offsets for v in required):raise RuntimeError('Start-neighborhood cases missing; no writes')
    if sorted(row['gain_scale'] for row in dynamic)!=[.5,1.,1.5]:raise RuntimeError('Gain cases missing; no writes')
    for row in dynamic:
        values=np.array([row[k] for k in ('max_tracking_rad','max_self_overlap_mm','max_support_overlap_mm')])
        if (row['finite'] is not True or not np.isfinite(values).all() or np.any(values<0)
                or values[0]>.12 or not self_pass(row,dynamic=True) or values[2]>.15):
            raise RuntimeError('Native dynamics screen rejected; no writes')
    return r
