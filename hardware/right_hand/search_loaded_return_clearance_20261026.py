"""Compare small EMPTY-only thumb-clearance candidates without any devices."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import numpy as np,mujoco
from mujoco_playground import registry
import screen_loaded_thumb_rotation_20261026 as screen
from loaded_thumb_rotation_plan_20261026 import FIXTURE,LoadedThumbRotationPlan,TURN_PHASE,RETURN_PHASE,motion_goals
from loaded_quarter_trial_plan_20261025 import empty_return_goals
from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan,TURN_PHASE as EMPTY_TURN
from hand_motor_mapping import ticks_to_sim
from book_grasp_trial_plan import RING_RELEASE
from camera_gap_native_policy_20261026 import add_stress_holds

def candidate(original,motor_id,delta):
    points=original.copy();ramp=np.minimum(np.arange(len(points)),np.arange(len(points))[::-1])
    points[:,motor_id-1]+=np.sign(delta)*np.minimum(abs(delta),ramp)
    out=[points[0].copy()]
    for a,b in zip(points,points[1:]):
        n=max(1,int(np.ceil(np.max(abs(b-a))/2)))
        out.extend(np.rint(a+(b-a)*k/n).astype(np.int64) for k in range(1,n+1))
    return np.asarray(out)

def main():
    p=LoadedThumbRotationPlan(FIXTURE/'request.json');raw=p.source_position()
    proof=json.loads((FIXTURE/'screen.json').read_bytes())
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=screen.build_cad(env,FIXTURE/'return_search_scene.xml',book_projection_mm=-10.);screen.correct_model(m,raw);m.opt.timestep=.005
    m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    inspect_raw=screen.cached_contacts(m,hand,{m.geom(n).id for n in ('support_book','support_glass','support_wood')})
    def inspect(d,phase):
        _,pairs=inspect_raw(d)
        pairs=screen.classify_self_contacts(pairs)
        pairs=screen.classify_photo_initial(pairs,initial_release=phase in ('initial_hold',RING_RELEASE))
        pairs=screen.classify_slow_seam(pairs);pairs=screen.classify_return_corner(pairs,phase=phase)
        return screen.classify_initial_palm(pairs,enabled=True,initial_release=phase==RING_RELEASE,phase=phase)
    bound=next(r['initial_self_contacts_mm'] for r in proof['dynamic_sensitivity'] if r['book_front_mm']==-10 and r['gain_scale']==.5)
    base=ThumbRootEmptyPlan.plan(p,raw);stop=next(i for i,(phase,q) in enumerate(base) if phase==EMPTY_TURN);base=base[:stop]
    anchor=int(np.argmin(np.max(abs(motion_goals()-raw),axis=1)));original=empty_return_goals(anchor)
    results=[]
    for motor,delta in ((4,-2),(4,-4),(4,-8),(2,-2),(2,2),(3,-2),(3,2),(1,2),(1,4)):
        path=candidate(original,motor,delta);d=mujoco.MjData(m);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
        peaks={}
        for q in path:
            d.qpos[:16]=ticks_to_sim(q);mujoco.mj_forward(m,d)
            for pair,depth in inspect(d,RETURN_PHASE).items():peaks[pair]=max(peaks.get(pair,0.),depth)
        result=dict(motor_id=motor,delta_ticks=delta,static_peaks_mm=peaks)
        if max(peaks.values(),default=0.)>1e-6:
            results.append(result);print(json.dumps(result),flush=True);continue
        rows=[];inserted=False
        for phase,q in base:
            if phase==RETURN_PHASE:
                if not inserted:rows.extend((phase,v.copy()) for v in path[1:]);inserted=True
            else:rows.append((phase,q.copy()))
        m.actuator_gainprm[:,0]*=.5;m.actuator_biasprm[:,1]*=.5
        worst=0.;first=None
        for stress in (False,True):
            d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
            trajectory=[('initial_hold',ticks_to_sim(raw),.5)]+[(phase,ticks_to_sim(q),.075) for phase,q in rows]
            if stress:trajectory,_=add_stress_holds(trajectory,rows)
            elapsed=0.
            for phase,angles,duration in trajectory:
                d.ctrl[:]=angles
                for _ in range(round(duration/.005)):
                    mujoco.mj_step(m,d);elapsed+=.005;pairs=inspect(d,phase)
                    excess=max((max(0.,depth-(bound.get(pair,0.) if phase in ('initial_hold',RING_RELEASE) else 0.)) for pair,depth in pairs.items()),default=0.)
                    worst=max(worst,excess)
                    if excess>1e-6 and first is None:first=dict(time=elapsed,phase=phase,pairs=pairs,stress=stress)
        m.actuator_gainprm[:,0]*=2;m.actuator_biasprm[:,1]*=2
        result.update(dynamic_unapproved_max_mm=worst,first_bad=first)
        results.append(result);print(json.dumps(result),flush=True)
        if worst<=1e-6:break
    (FIXTURE/'return_clearance_search.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
if __name__=='__main__':main()
