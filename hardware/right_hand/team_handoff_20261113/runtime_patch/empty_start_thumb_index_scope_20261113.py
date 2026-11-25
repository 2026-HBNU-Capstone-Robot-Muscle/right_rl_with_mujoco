"""One user-approved initial EMPTY model overlap; shapes and forces stay intact."""
import math
from new_pair_grasp_plan_20261027 import ENTRY,ROOTS,OPEN,SETTLE
from horizontal_empty_recovery_20261027 import RING_CLEAR
from calibrated_empty_ring_margin_20261028 import PHASE as RING_MARGIN
from rewired_initial_pair_scope_20261027 import InitialPairScope as LegacyScope

PAIR='if_bs_collision_1|th_tip'
EPS=1e-6
POLICY=dict(version='initial_empty_thumb_index_non_deepening_v1_20261113',pair=PAIR,
            maximum_initial_model_depth_mm=1.75,maximum_open_goals_to_clear=48,
            maximum_depth_increase_mm=EPS,empty_only=True,loaded_contacts_unchanged=True,
            geometry_and_contact_forces_unchanged=True,physical_gap_calibrated=False,
            approval='User requested allowance for the observed initial model overlap on 2026-11-12/13')

class EmptyInitialScope:
    def __init__(self,initial,enabled,*,mode):
        if mode not in ('static','dynamic'):raise ValueError('Scope needs explicit sampling mode')
        self.mode=mode;self.depth=float(initial.get(PAIR,0.));self.active=self.depth>EPS
        self.legacy=LegacyScope({k:v for k,v in initial.items() if k!=PAIR},enabled)
        self.maximum_raw=max(initial.values(),default=0.);self.peak=self.depth
        self.open_calls=0;self.last_positive_open=0;self.violation=False;self.settled=False
    def __getattr__(self,name):return getattr(self.legacy,name)
    @property
    def valid(self):
        return bool(self.legacy.valid and math.isfinite(self.depth) and 0<=self.depth<=POLICY['maximum_initial_model_depth_mm']
                    and self.peak<=self.depth+EPS and not self.violation and self.settled)
    def filtered(self,phase,pairs):
        result=dict(pairs);depth=float(pairs.get(PAIR,0.));self.peak=max(self.peak,depth)
        self.maximum_raw=max(self.maximum_raw,max(pairs.values(),default=0.))
        if phase==OPEN:
            self.open_calls+=1
            if depth>EPS:self.last_positive_open=self.open_calls
        deadline=POLICY['maximum_open_goals_to_clear']*(15 if self.mode=='dynamic' else 1)
        allowed=phase in ('initial_hold',ENTRY,ROOTS,RING_CLEAR,RING_MARGIN,OPEN)
        if phase==OPEN and self.open_calls>deadline:allowed=False
        if phase==SETTLE:self.settled=True
        if depth>EPS and (not allowed or not self.active):self.violation=True
        if self.active and allowed and depth<=self.depth+EPS:result.pop(PAIR,None)
        return self.legacy.filtered(phase,result)
    def review(self):
        return dict(policy_version=POLICY['version'],mode=self.mode,enabled=self.active,
                    initial_mm=self.depth,maximum_mm=self.peak,increase_mm=max(0.,self.peak-self.depth),
                    last_positive_open_call=self.last_positive_open,settle_observed=self.settled,
                    out_of_scope_contact=self.violation,passed=self.valid)

def validate_report(empty):
    rows=empty.get('static_cases',[])+empty.get('dynamic_cases',[])
    present=['thumb_index_initial_scope' in row for row in rows]
    if not any(present):return dict(enabled=False,legacy_zero_contact_review=True,policy=dict(POLICY))
    if not all(present):raise RuntimeError('Incomplete initial thumb/index scope evidence')
    for row in rows:
        r=row['thumb_index_initial_scope'];mode='static' if 'offset_ticks' in row else 'dynamic'
        cap=48*(15 if mode=='dynamic' else 1)
        if (r.get('policy_version')!=POLICY['version'] or r.get('mode')!=mode or r.get('passed') is not True
            or r.get('settle_observed') is not True or r.get('out_of_scope_contact') is not False):
            raise RuntimeError('Initial thumb/index scope incomplete or exceeded')
        for key,limit in [('initial_mm',1.75),('maximum_mm',1.75),('increase_mm',EPS),('last_positive_open_call',cap)]:
            value=r.get(key)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<=value<=limit:
                raise RuntimeError('Unbounded initial thumb/index evidence: '+key)
        if r['maximum_mm']>r['initial_mm']+EPS:raise RuntimeError('Initial overlap grew during recovery')
    return dict(enabled=any(row['thumb_index_initial_scope']['enabled'] for row in rows),policy=dict(POLICY),
                maximum_initial_mm=max(row['thumb_index_initial_scope']['initial_mm'] for row in rows),
                maximum_increase_mm=max(row['thumb_index_initial_scope']['increase_mm'] for row in rows),
                all_cases_cleared_by_open_deadline=True,cases=len(rows))
