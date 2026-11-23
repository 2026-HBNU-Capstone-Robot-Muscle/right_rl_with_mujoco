"""Bounded separation of an already-present EMPTY IF tip/MF middle overlap."""
import math
from empty_start_thumb_index_scope_20261113 import EmptyInitialScope as Base,validate_report as base_validate
from new_pair_grasp_plan_20261027 import ENTRY,SETTLE

PAIR='if_ds_collision_1|mf_md_collision_5';EPS=1e-6
POLICY=dict(version='initial_interrupted_IF_tip_MF_middle_escape_v1_20261116',pair=PAIR,
    maximum_initial_mm=.55,maximum_ENTRY_goals_to_clear=32,maximum_increase_mm=EPS,
    measured_start_nominal_model_mm=.141692,plus4tick_static_case_mm=.518370,
    empty_recovery_only=True,geometry_and_contact_forces_unchanged=True,
    physical_gap_calibrated=False)

class InitialScope:
    def __init__(self,initial,enabled,*,mode):
        self.base=Base({k:v for k,v in initial.items() if k!=PAIR},enabled,mode=mode)
        self.mode=mode;self.depth=float(initial.get(PAIR,0.));self.peak=self.depth
        self.calls=0;self.violation=False;self.settled=False;self.last_positive=0;self.cleared=self.depth<=EPS
    def __getattr__(self,name):return getattr(self.base,name)
    @property
    def maximum_raw(self):return max(self.base.maximum_raw,self.peak)
    @property
    def valid(self):
        return bool(self.base.valid and math.isfinite(self.depth) and 0<=self.depth<=POLICY['maximum_initial_mm']
            and self.peak<=self.depth+EPS and not self.violation and self.settled)
    def filtered(self,phase,pairs):
        result=dict(pairs);depth=float(result.get(PAIR,0.));self.peak=max(self.peak,depth)
        if phase==ENTRY:
            self.calls+=1
            if depth>EPS:self.last_positive=self.calls
        cap=32*(15 if self.mode=='dynamic' else 1)
        allowed=phase=='initial_hold' or phase==ENTRY and self.calls<=cap
        if depth>EPS:
            if self.depth>EPS and not self.cleared and allowed and depth<=self.depth+EPS:result.pop(PAIR,None)
            else:self.violation=True
        else:self.cleared=True
        if phase==SETTLE:self.settled=True
        return self.base.filtered(phase,result)
    def review(self):
        r=self.base.review();r['passed']=self.valid
        r['interrupted_index_middle_scope']=dict(policy_version=POLICY['version'],mode=self.mode,
            initial_mm=self.depth,maximum_mm=self.peak,increase_mm=max(0.,self.peak-self.depth),
            last_positive_ENTRY_call=self.last_positive,settle_observed=self.settled,
            out_of_scope_contact=self.violation,passed=self.valid)
        return r

def validate_report(empty):
    old=base_validate(empty);rows=empty['static_cases']+empty['dynamic_cases'];active=[]
    for row in rows:
        r=row.get('thumb_index_initial_scope',{}).get('interrupted_index_middle_scope')
        if r is None:
            # Previously accepted zero/old-pair-only evidence remains valid.
            if row['max_self_mm']>EPS:raise RuntimeError('Missing interrupted-pair evidence')
            continue
        mode='static' if 'offset_ticks' in row else 'dynamic';cap=32*(15 if mode=='dynamic' else 1)
        if (r.get('policy_version')!=POLICY['version'] or r.get('mode')!=mode
            or r.get('passed') is not True or r.get('out_of_scope_contact') is not False
            or r.get('settle_observed') is not True):raise RuntimeError('Interrupted contact did not clear')
        for key,limit in (('initial_mm',POLICY['maximum_initial_mm']),('maximum_mm',POLICY['maximum_initial_mm']),('increase_mm',EPS),('last_positive_ENTRY_call',cap)):
            x=r.get(key)
            if isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) or not 0<=x<=limit:
                raise RuntimeError('Interrupted contact scope exceeded: '+key)
        if r['maximum_mm']>r['initial_mm']+EPS:raise RuntimeError('Interrupted contact deepened')
        active.append(r)
    return dict(thumb_index=old,interrupted_pair_policy=POLICY,
        interrupted_pair_cases=len(active),maximum_initial_mm=max([r['initial_mm'] for r in active]+[0.]))
