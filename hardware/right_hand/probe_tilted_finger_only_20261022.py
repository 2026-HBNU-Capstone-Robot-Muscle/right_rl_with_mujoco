"""Offline restrained-joint variants of the historical intended vertical motion."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from sac_forward_geometry import measure,eligible

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'prepared_cube_review_20261022'


def main():
    obs=json.loads((OUT/'placement_review.json').read_text(encoding='utf-8'))
    scan=json.loads((OUT/'support_settling_scan.json').read_bytes())
    old=json.loads((ROOT/'full_mapping_pose_review_20261019/inward_quantized/housing_sensitivity/path_OFFLINE_ONLY.json').read_bytes())
    raw=np.array([old['start_ticks']]+old['goals_id_order']);start=np.array(obs['placement_goal_ticks'])
    raw=np.rint(raw+np.maximum(0,1-np.arange(len(raw))/20)[:,None]*(start-raw[0])[None,:]).astype(int)
    variants={
        'thumb_fixed':(1,2,3,4),
        'thumb_ring_fixed':(1,2,3,4,13,14,15,16),
        'thumb_ring_rot_fixed':(1,2,3,4,6,10,13,14,15,16),
        'thumb_tip_fixed':(1,),
        'thumb_tip_ring_fixed':(1,13,14,15,16),
        'thumb_tip_ring_rot_fixed':(1,6,10,13,14,15,16),
    }
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);set_cube_physics(m,side_m=.055,mass_kg=.100)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_glass','support_wood')};cube=m.geom('cube').id
    def collision(d):
        amax=smax=0.;pair=None
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);pen=max(0,-float(c.dist)*1000)
            if int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand and pen>amax:
                amax=pen;pair=[m.geom(a).name,m.geom(b).name]
            elif (a in support or b in support) and a!=cube and b!=cube:smax=max(smax,pen)
        return amax,smax,pair
    results=[]
    for name,fixed in variants.items():
        candidate=raw.copy();candidate[:,np.array(fixed)-1]=start[np.array(fixed)-1]
        goals=ticks_to_sim(candidate);d=mujoco.MjData(m);static=0.;pair=None
        for q in goals:
            d.qpos[:16]=q;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
            depth,_,p=collision(d)
            if depth>static:static=depth;pair=p
        trials=[]
        for speed in (1,1.5):
            for ri,reset in enumerate(scan['dynamic_holds'][:1]):
                d=mujoco.MjData(m);d.qpos[:]=reset['qpos'];d.qpos[:16]=ticks_to_sim(obs['placement_position_ticks'])
                d.ctrl[:]=goals[0];mujoco.mj_forward(m,d);origin=d.qpos[-7:].copy()
                depth=sup=error=0.;best=streak=0;trace=[]
                for k,g in enumerate([goals[0]]*20+list(goals)+[goals[-1]]*20):
                    d.ctrl[:]=g
                    for _ in range(round(.05*speed/m.opt.timestep)):
                        mujoco.mj_step(m,d);a,b,p=collision(d);depth=max(depth,a);sup=max(sup,b)
                    error=max(error,float(np.max(abs(d.qpos[:16]-g))))
                    tip,axis,te=map(float,measure(origin[-4:],d.qpos[-4:]));shift=float(np.linalg.norm(d.qpos[-7:-4]-origin[:3]))
                    good=bool(eligible(tip,axis,te,shift));streak=streak+1 if good else 0;best=max(best,streak)
                    trace.append(dict(time_s=(k+1)*.05*speed,qpos=d.qpos.tolist()))
                trials.append(dict(speed=speed,reset=ri,tip_deg=float(np.degrees(tip)),off_axis_deg=float(np.degrees(axis)),
                    target_error_deg=float(np.degrees(te)),shift_mm=shift*1000,max_tracking_rad=error,
                    max_self_mm=depth,max_support_mm=sup,target_hold_s=best*.05*speed,trace=trace))
        row=dict(name=name,held_ids=list(fixed),static_self_mm=static,static_pair=pair,
            start_ticks=candidate[0].tolist(),goals_id_order=candidate[1:].tolist(),trials=trials)
        results.append(row)
        print(json.dumps(dict(name=name,static_self_mm=static,static_pair=pair,
            trials=[{k:v for k,v in r.items() if k!='trace'} for r in trials])),flush=True)
    (OUT/'finger_only_probe.json').write_text(json.dumps(dict(status='offline_finger_only_probe',
        model_manifest=manifest,hardware_execution_allowed=False,motor_commands_sent=0,live_sac=False,
        seed_registration_verified=False,variants=results),indent=2))


if __name__=='__main__':main()
