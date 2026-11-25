"""Find a genuine clearance waypoint from the native weak-response state.

Replays the existing prefix once and tests alternatives at unchanged limits.
No device imports or writes. This diagnostic cannot authorize execution.
"""
import json
import numpy as np
import mujoco
import screen_thumb_clear_root_empty_20261026 as screen
from thumb_clear_root_empty_plan_20261026 import FIXTURE,ThumbRootEmptyPlan,THUMB_LIFT,THUMB_DELTA,ROOT_EXTRA,finish_goals
from grasp_recovery_plan import interpolate
from slow_thumb_seam_resolution_20261026 import classify
from observed_book_model_policy import classify_self_contacts

class PrefixCaptured(Exception):pass
def main():
    plan=ThumbRootEmptyPlan(FIXTURE/'request.json');rows=plan.plan(plan.source_position())
    prefix=next(i for i,(p,q) in enumerate(rows) if p==THUMB_LIFT)
    capture_time=.5+prefix*.075;captured={};original=mujoco.mj_step;prefix_peaks={};inspector=[None]
    def step(m,d):
        if d.time>=capture_time-1e-7:
            captured.update(model=m,time=float(d.time),**{n:np.array(getattr(d,n),copy=True) for n in ('qpos','qvel','act','ctrl','qacc_warmstart','mocap_pos','mocap_quat')})
            raise PrefixCaptured()
        result=original(m,d)
        if d.time>5:
            if inspector[0] is None:
                hand={m.body('palm').id}
                for b in range(m.nbody):
                    if int(m.body_parentid[b]) in hand:hand.add(b)
                support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
                inspector[0]=screen.cached_contacts(m,hand,support)
            _,pairs=inspector[0](d);pairs=classify(classify_self_contacts(pairs))
            for pair,depth in pairs.items():
                if depth>prefix_peaks.get(pair,{}).get('depth_mm',0):prefix_peaks[pair]=dict(depth_mm=depth,time_s=float(d.time))
        return result
    screen.FRONTS=(-10.,);screen.SCALES=(.5,);screen.offsets=lambda:[np.zeros(16,dtype=int)]
    screen.mujoco.mj_step=step
    try:screen.main(FIXTURE/'request.json',FIXTURE/'prefix_only_not_a_proof.json')
    except PrefixCaptured:pass
    finally:screen.mujoco.mj_step=original
    if not captured:raise RuntimeError('No native prefix state captured')
    m=captured['model'];palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    inspect=screen.cached_contacts(m,hand,support);results=[]
    for amount in (128,144,160,192,224):
        start=finish_goals()[-1].copy();lift=start.copy();lift[3]-=amount
        thumb=start.copy();thumb[:4]+=THUMB_DELTA;end=thumb.copy();end[[7,11]]+=ROOT_EXTRA
        goals=[];interpolate(goals,start,lift,'lift');interpolate(goals,lift,thumb,'retreat');interpolate(goals,thumb,end,'roots')
        goals.extend(('hold',end.copy()) for _ in range(80));d=mujoco.MjData(m)
        for n in ('qpos','qvel','act','ctrl','qacc_warmstart','mocap_pos','mocap_quat'):getattr(d,n)[:]=captured[n]
        d.time=captured['time'];maxself=maxsupport=maxtrack=0.;peaks={}
        for phase,raw in goals:
            goal=screen.ticks_to_sim(raw);d.ctrl[:]=goal
            for _ in range(15):
                original(m,d);a,b=inspect(d);b=classify(classify_self_contacts(b))
                maxsupport=max(maxsupport,max(a.values(),default=0));maxself=max(maxself,max(b.values(),default=0));maxtrack=max(maxtrack,float(np.max(abs(d.qpos[:16]-goal))))
                for pair,depth in b.items():
                    if depth>peaks.get(pair,{}).get('depth_mm',0):peaks[pair]=dict(depth_mm=depth,phase=phase,time_s=float(d.time))
        result=dict(cmc_lift_ticks=amount,max_self_mm=maxself,max_support_mm=maxsupport,max_tracking_rad=maxtrack,
                    contact_peaks=peaks,clear=bool(maxself<=1e-6 and maxsupport<=.15 and maxtrack<=.12))
        results.append(result);print('WAYPOINT',json.dumps(result),flush=True)
    (FIXTURE/'waypoint_diagnostic_with_prefix.json').write_text(json.dumps(dict(prefix_count=prefix,captured_time=captured['time'],prefix_contact_peaks=prefix_peaks,cases=results,hardware_authorized=False,hardware_commands_sent=0),indent=2),encoding='utf-8')
    print('PREFIX CONTACTS',json.dumps(prefix_peaks),flush=True)
if __name__=='__main__':main()
