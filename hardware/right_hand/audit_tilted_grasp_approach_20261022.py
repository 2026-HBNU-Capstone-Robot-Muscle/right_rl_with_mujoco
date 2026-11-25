"""Offline approach paths to a tentative cube grasp. No hardware interfaces."""
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

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'prepared_cube_review_20261022'


def main():
    obs=json.loads((OUT/'placement_review.json').read_text(encoding='utf-8'))
    scan=json.loads((OUT/'support_settling_scan.json').read_bytes())
    ik=json.loads((OUT/'grasp_geometry_multiface_search.json').read_bytes())
    start=np.array(obs['placement_goal_ticks']);target0=np.array(ik['rows'][0]['raw_ticks'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);set_cube_physics(m,side_m=.055,mass_kg=.100)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_glass','support_wood')};cube=m.geom('cube').id
    bounds=m.jnt_range[m.actuator_trnid[:,0]].copy();cb=m.actuator_ctrlrange.copy()
    def contact(d):
        dep=np.zeros(3)
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);depth=max(0,-float(c.dist)*1000)
            if cube in (a,b):dep[2]=max(dep[2],depth)
            elif a in support or b in support:dep[1]=max(dep[1],depth)
            elif int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:dep[0]=max(dep[0],depth)
        return dep
    cases=[]
    for relief in (32,64,96,128):
        target=target0.copy();target[13]+=relief
        for order in ('simultaneous','thumb_first','fingers_first'):
            raw=[start.copy()];last=start.copy();groups=[np.array([13])]
            groups+=([np.arange(16)] if order=='simultaneous' else
                [np.arange(4),np.arange(4,16)] if order=='thumb_first' else [np.arange(4,16),np.arange(4)])
            for group in groups:
                end=last.copy();end[group]=target[group]
                n=max(20,int(np.ceil(np.max(abs(end-last))/6)))
                for k in range(1,n+1):raw.append(np.rint(last+(end-last)*k/n).astype(int))
                last=end
            goals=ticks_to_sim(np.array(raw));static=np.zeros(3)
            for goal in goals:
                d=mujoco.MjData(m);d.qpos[:16]=goal;d.qpos[-7:]=scan['dynamic_holds'][0]['qpos'][-7:];mujoco.mj_forward(m,d)
                static=np.maximum(static,contact(d))
            violation=bool(np.any(goals<bounds[:,0]) or np.any(goals>bounds[:,1]) or np.any(goals<cb[:,0]) or np.any(goals>cb[:,1]))
            row=dict(relief_ticks=relief,order=order,static_self_mm=float(static[0]),static_support_mm=float(static[1]),
                static_cube_mm=float(static[2]),outside_native_range=violation,samples=len(raw),
                start_ticks=raw[0].tolist(),goals_id_order=np.array(raw[1:]).tolist(),trials=[])
            if not violation and static[0]<.01 and static[1]<.15 and static[2]<1:
                for ri,reset in enumerate(scan['dynamic_holds']):
                    d=mujoco.MjData(m);d.qpos[:]=reset['qpos'];d.qpos[:16]=ticks_to_sim(obs['placement_position_ticks'])
                    d.ctrl[:]=goals[0];mujoco.mj_forward(m,d);origin=d.qpos[-7:].copy()
                    peak=np.zeros(3);tracking=0;trace=[]
                    for k,goal in enumerate([goals[0]]*20+list(goals)+[goals[-1]]*20):
                        d.ctrl[:]=goal
                        for _ in range(round(.05/m.opt.timestep)):
                            mujoco.mj_step(m,d);peak=np.maximum(peak,contact(d))
                        tracking=max(tracking,float(np.max(abs(d.qpos[:16]-goal))))
                        trace.append(dict(time_s=(k+1)*.05,qpos=d.qpos.tolist()))
                    touched=[]
                    for c in d.contact:
                        a,b=int(c.geom1),int(c.geom2)
                        if cube in (a,b):touched.append(m.geom(b if a==cube else a).name)
                    row['trials'].append(dict(reset=ri,max_self_mm=float(peak[0]),max_support_mm=float(peak[1]),
                        max_cube_mm=float(peak[2]),max_tracking_rad=tracking,
                        cube_shift_mm=float(np.linalg.norm(d.qpos[-7:-4]-origin[:3])*1000),
                        final_cube_contacts=sorted(set(touched)),trace=trace))
            cases.append(row)
            print(json.dumps({k:v for k,v in row.items() if k not in ('goals_id_order','start_ticks','trials')} | dict(
                trials=[{k:v for k,v in t.items() if k!='trace'} for t in row['trials']])),flush=True)
    assert np.array_equal(bounds,m.jnt_range[m.actuator_trnid[:,0]]) and np.array_equal(cb,m.actuator_ctrlrange)
    (OUT/'grasp_approach_screen.json').write_text(json.dumps(dict(status='offline_grasp_approach_screen',
        model_manifest=manifest,hardware_execution_allowed=False,motor_commands_sent=0,live_sac=False,
        registration_verified=False,cases=cases),indent=2))


if __name__=='__main__':main()
