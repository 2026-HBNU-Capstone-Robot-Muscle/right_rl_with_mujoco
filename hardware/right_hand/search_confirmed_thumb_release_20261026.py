"""Offline prefix search; keep every contact force and monotonic release rule."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import itertools,json
import numpy as np
import mujoco
from mujoco_playground import registry
from thumb_clear_root_empty_plan_20261026 import ROOT,FIXTURE,ThumbRootEmptyPlan
from screen_thumb_clear_root_empty_20261026 import cached_contacts
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from observed_book_model_policy import classify_self_contacts
from support_photo_contact_resolution_20261025 import classify as photo
from slow_thumb_seam_resolution_20261026 import classify as seam
from book_empty_preparation_plan import offsets,ALLOWED_SUPPORT_PAIRS
from interrupted_thumb_contact_policy_20261026 import metadata
from initial_contact_escape import ContactEscape
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
PAIR='palm_collision_3|th_tip'
OUT=FIXTURE/'prefix_search'
def main():
    OUT.mkdir(exist_ok=True)
    p=ThumbRootEmptyPlan(FIXTURE/'request.json');raw=p.source_position()
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build(env,OUT/'scene.xml',book_projection_mm=-5.);correct_model(m,raw)
    m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    inspect=cached_contacts(m,hand,{m.geom(n).id for n in ('support_book','support_glass','support_wood')})
    lo,hi=limits();starts=[raw+off for off in offsets()]
    def contact(q):
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
        mujoco.mj_forward(m,d);sp,ss=inspect(d)
        return sp,seam(photo(classify_self_contacts(ss),initial_release=True))
    initial=[contact(a) for a in starts]
    parent=set(metadata(raw)['additional_initial_pairs'])|{'palm_collision_7|th_tip',PAIR}
    parent|=set().union(*(set(v[1]) for v in initial))
    results=[]
    candidates=[(0,-n,0,0) for n in (16,32,64)]
    candidates += [tuple(n*v for v in direction) for n in (16,32,64)
        for direction in itertools.product((-1,0,1),repeat=4)
        if any(direction) and direction[1]<=0 and sum(abs(v) for v in direction)<=2]
    for delta_tuple in dict.fromkeys(candidates):
        delta=np.zeros(16,dtype=int);delta[:4]=delta_tuple;count=int(max(abs(delta))//2)
        cases=[];failure={}
        for a,(sp0,ss0) in zip(starts,initial):
            if np.any(a+delta<lo) or np.any(a+delta>hi):failure=dict(reason='range',bad_ids=(np.flatnonzero((a+delta<lo)|(a+delta>hi))+1).tolist());break
            self_escape=ContactEscape(ss0,parent,4.,count,monotonic=True)
            support_escape=ContactEscape(sp0,ALLOWED_SUPPORT_PAIRS,5.,count,monotonic=True)
            clear=None
            for k in range(1,count+1):
                sp,ss=contact(np.rint(a+delta*k/count).astype(int))
                self_escape.observe(ss,k)
                # Supports need not disappear until the later root lift.
                support_escape.observe(sp,0)
                if self_escape.unapproved>1e-6 or support_escape.unapproved>1e-6:
                    failure=dict(reason='contact',at=k,offset=(a-raw).tolist(),initial_self=ss0,self_now=ss,
                        self_unapproved=self_escape.unapproved,support_unapproved=support_escape.unapproved)
                    break
                if not ss and clear is None:clear=k
            else:
                if clear is not None:cases.append(clear);continue
            break
        passed=len(cases)==len(starts)
        results.append(dict(delta=list(delta_tuple),passed=passed,cases=len(cases),goals=count,
            worst_clear_goal=max(cases,default=None),failure=failure))
        if passed:print('PASS',json.dumps(results[-1]),flush=True)
    # A short curved path can avoid grazing a neighbouring face when a straight
    # interpolation cannot. Every step is tested for every +/-4tick start.
    states=[(sp,ss) for sp,ss in initial];delta=np.zeros(16,dtype=int);waypoints=[];greedy=[]
    moves=[]
    for move in itertools.product((-2,0,2),repeat=4):
        if any(move) and move[1]<=0:
            dlt=np.zeros(16,dtype=int);dlt[:4]=move;moves.append(dlt)
    for step in range(64):
        best=None
        for move in moves:
            proposed=delta+move
            if max(abs(proposed))>96:continue
            if any(np.any(a+proposed<lo) or np.any(a+proposed>hi) for a in starts):continue
            next_states=[];cost=0.
            for a,(sp0,ss0),(sp_prev,ss_prev) in zip(starts,initial,states):
                sp,ss=contact(a+proposed)
                if any(depth>ss0.get(pair,0.)+1e-6 or depth>ss_prev.get(pair,0.)+1e-6 for pair,depth in ss.items()):break
                if any(depth>sp0.get(pair,0.)+1e-6 or depth>sp_prev.get(pair,0.)+1e-6 for pair,depth in sp.items()):break
                cost+=sum(ss.values());next_states.append((sp,ss))
            if len(next_states)==len(starts):
                score=cost+1e-7*sum(abs(proposed))
                if best is None or score<best[0]:best=(score,proposed,next_states)
        if best is None:
            print('GREEDY_BLOCKED',step,delta[:4].tolist(),flush=True);break
        _,delta,states=best;waypoints.append(delta[:4].tolist());greedy.append(dict(step=step+1,delta=delta[:4].tolist(),cost=best[0]))
        if not any(ss for _,ss in states):
            print('GREEDY_PASS',len(waypoints),delta[:4].tolist(),flush=True);break
        if step%8==0:print('GREEDY',greedy[-1],flush=True)
    result=dict(source=str(FIXTURE/'request.json'),cases=len(starts),results=results,
        initial_pair3_max_mm=max(ss.get(PAIR,0.) for _,ss in initial),motor_commands_sent=0,
        greedy_waypoint_deltas=waypoints,greedy_log=greedy,greedy_cleared=bool(states) and not any(ss for _,ss in states))
    (OUT/'review.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('FIRST_FAILURES',json.dumps(results[:4]),flush=True)
    print('DONE',sum(r['passed'] for r in results),'/',len(results),flush=True)
if __name__=='__main__':main()
