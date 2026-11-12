"""MuJoCo-only replay of the failed saved request; no hardware imports."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from supported_yaw_empty_plan import SupportedYawEmptyPlan
from grasp_place_plan import endpoints
from grasp_recovery_plan import interpolate,THUMB_NATIVE_RECOVERY_FLOOR,THUMB_CLEARANCE_TICKS
from initial_contact_escape import ContactEscape

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'supported_yaw_empty_20261022_155659_749370'

def release_variant(raw, tip_extension):
    opened,_=endpoints();a=raw.copy();rows=[]
    release=a.copy();release[14]=min(a[14],opened[14])
    release[3]=max(a[3],THUMB_NATIVE_RECOVERY_FLOOR)
    release[12]=a[12]-tip_extension
    a=interpolate(rows,a,release,'stage_0_empty_recover')
    thumb=a.copy();delta=opened[:4]-a[:4]
    alpha=min(1.,THUMB_CLEARANCE_TICKS/max(1,int(max(abs(delta)))))
    thumb[:4]=np.rint(a[:4]+delta*alpha).astype(np.int64)
    a=interpolate(rows,a,thumb,'stage_1_thumb_clearance')
    interpolate(rows,a,opened,'stage_2_empty_open')
    return rows

def main():
    plan=SupportedYawEmptyPlan(FOLDER/'fresh_off_request.json')
    raw=plan.source_position()
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build_with_middle_thumb_hulls(env)
    cube=m.geom('cube').id;m.geom_contype[cube]=m.geom_conaffinity[cube]=0
    palm=m.body('palm').id;hand={palm}
    for body in range(m.nbody):
        if int(m.body_parentid[body]) in hand:hand.add(body)
    def pairs(d):
        found={}
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand and c.dist<0:
                key='|'.join(sorted((m.geom(a).name,m.geom(b).name)))
                found[key]=max(found.get(key,0),-float(c.dist)*1000)
        return found
    native_gain=m.actuator_gainprm.copy();native_bias=m.actuator_biasprm.copy()
    outputs=[]
    offsets=[np.zeros(16,dtype=int),np.full(16,-4),np.full(16,4)]
    for i in range(16):
        for sign in (-1,1):
            v=np.zeros(16,dtype=int);v[i]=4*sign;offsets.append(v)
    offsets.extend(np.random.default_rng(20261021).integers(-4,5,(32,16)))
    for extension in (0,8,16,32,64):
        static=[]
        for offset in offsets:
            start=raw+offset;d=mujoco.MjData(m)
            d.qpos[:16]=ticks_to_sim(start);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
            mujoco.mj_forward(m,d)
            escape=ContactEscape(pairs(d),plan.INITIAL_SELF_ESCAPE_PAIRS,None,40,monotonic=True)
            rows=release_variant(start,extension)
            travel=None
            for index,(phase,q) in enumerate(rows,1):
                d.qpos[:16]=ticks_to_sim(q);mujoco.mj_forward(m,d)
                escape.observe(pairs(d),index)
                if travel is None and escape.complete_at is not None:travel=int(start[14]-q[14])
            static.append(dict(offset=offset.tolist(),travel=travel,**escape.result()))
        summary=dict(tip_extension_ticks=extension,
            static_failures=sum(s['self_escape_max_unapproved_overlap_mm']>1e-6 or not s['self_escape_complete'] for s in static),
            maximum_clearance_travel=max(s['travel'] or 0 for s in static),
            maximum_unapproved=max(s['self_escape_max_unapproved_overlap_mm'] for s in static))
        print('STATIC',json.dumps(summary),flush=True)
        outputs.append(dict(**summary,static=static))
    for extension,gain in ((e,g) for e in (0,8,16,32,64) for g in (.5,1.,1.5)):
        m.actuator_gainprm[:]=native_gain;m.actuator_biasprm[:]=native_bias
        m.actuator_gainprm[:,0]*=gain;m.actuator_biasprm[:,1]*=gain
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw)
        d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];d.ctrl[:]=ticks_to_sim(raw)
        mujoco.mj_forward(m,d);initial=pairs(d);previous=initial
        complete=None;events=[]
        path=[('initial_hold',raw)]*10+release_variant(raw,extension)
        for sample,(phase,q) in enumerate(path):
            if d.time>3.5:break
            d.ctrl[:]=ticks_to_sim(q)
            for _ in range(round(.05/m.opt.timestep)):
                mujoco.mj_step(m,d);current=pairs(d)
                if not current and complete is None:complete=float(d.time)
                bad={k:v for k,v in current.items() if k not in initial or v>initial[k]+1e-6 or d.time>2.5}
                if bad:
                    events.append(dict(t=float(d.time),phase=phase,sample=sample,contacts=bad,
                        goal=q.tolist(),actual_sim=d.qpos[:16].tolist()))
                previous=current
        row=dict(tip_extension_ticks=extension,gain=gain,initial=initial,first_clear=complete,violations=events)
        outputs.append(row)
        print(json.dumps(dict(tip_extension_ticks=extension,gain=gain,initial=initial,first_clear=complete,
                              violations=len(events),first=events[:3],last=events[-1:])),flush=True)
    target=ROOT/'right_hand_rotation_20261022/supported_release_failure_dynamics.json'
    target.write_text(json.dumps(outputs,indent=2),encoding='utf-8')

if __name__=='__main__':main()
