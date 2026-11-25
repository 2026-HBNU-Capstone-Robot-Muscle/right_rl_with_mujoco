"""Compare bounded EMPTY release schedules, never relax collision checks."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from hand_motor_mapping import ticks_to_sim
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from slow_grasp_demo_plan import SlowGraspDemoPlan,OBSERVATION_PHASE
from supported_yaw_empty_plan import opening_rows
from grasp_place_plan import endpoints,LOADED_PHASE
from grasp_recovery_plan import interpolate

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'right_hand_rotation_20261023/slow_grasp_demo_fixture'


def main():
    plan=SlowGraspDemoPlan(FOLDER/'fresh_off_request.json');raw=plan.source_position()
    opened,closed=endpoints();base=opening_rows(raw,opened)
    base.extend([('placement_hold',opened.copy())]*40)
    interpolate(base,opened,closed,LOADED_PHASE)
    base.extend([(OBSERVATION_PHASE,closed.copy())]*60+[('final_hold',closed.copy())]*20)
    rows=base
    phase0=[q for phase,q in rows if phase=='stage_0_empty_recover']
    target=phase0[-1];prefix=rows
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build_with_middle_thumb_hulls(env)
    cube=m.geom('cube').id;m.geom_contype[cube]=m.geom_conaffinity[cube]=0
    palm=m.body('palm').id;hand={palm}
    for body in range(m.nbody):
        if int(m.body_parentid[body]) in hand:hand.add(body)
    gain=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();results=[]
    for variant in ('original','frontloaded_tip'):
        if variant=='original':route=prefix
        elif variant=='frontloaded_tip':
            route=[]
            for i,(phase,q) in enumerate(prefix,1):
                q=q.copy()
                if phase=='stage_0_empty_recover':
                    q[12]=raw[12]+np.clip(target[12]-raw[12],-2*i,2*i)
                route.append((phase,q))
        else:
            route=[];last_phase=None;last=None
            for phase,q in prefix:
                if last_phase=='stage_2_empty_open' and phase!=last_phase:
                    end=last.copy();end[12]=q[12]
                    for i in range(1,36):route.append(('stage_2_ring_tip_return',np.rint(last+(end-last)*i/35).astype(int)))
                q=q.copy()
                if phase in ('stage_2_empty_open',):q[12]=target[12]
                route.append((phase,q));last_phase=phase;last=q.copy()
        for g in (.5,1.,1.5):
            m.actuator_gainprm[:]=gain;m.actuator_biasprm[:]=bias
            m.actuator_gainprm[:,0]*=g;m.actuator_biasprm[:,1]*=g
            d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:]=[0,0,2,1,0,0,0]
            d.ctrl[:]=ticks_to_sim(raw);mujoco.mj_forward(m,d)
            def contacts():
                found={}
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    if int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand and c.dist<0:
                        key='|'.join(sorted((m.geom(a).name,m.geom(b).name)))
                        found[key]=max(found.get(key,0.),-float(c.dist)*1000)
                return found
            initial=contacts();unapproved=[];elapsed=0.;cleared=None;max_step=0;last=raw
            for phase,q in [('initial_hold',raw)]*10+route:
                max_step=max(max_step,int(max(abs(q-last))));last=q
                d.ctrl[:]=ticks_to_sim(q)
                for _ in range(round(.05/m.opt.timestep)):
                    mujoco.mj_step(m,d);elapsed+=m.opt.timestep
                    found=contacts()
                    if not found and cleared is None:cleared=elapsed
                    bad={key:depth for key,depth in found.items()
                         if key not in initial or depth>initial[key]+1e-6 or elapsed>2.5}
                    if bad and len(unapproved)<3:unapproved.append(dict(t=elapsed,phase=phase,contacts=bad))
            results.append(dict(variant=variant,gain=g,unapproved_first_samples=unapproved,
                first_clear_at_s=cleared,max_goal_step_ticks=max_step))
            print(json.dumps(results[-1]),flush=True)
    (FOLDER/'release_schedule_diagnostic.json').write_text(json.dumps(results,indent=2),encoding='utf-8')


if __name__=='__main__':main()
