"""Offline sensitivity of the slow, EMPTY-tested approach with a 55mm cube.

Seven provisional settled cube positions and three native servo gain scales.
This is a commissioning observation, not a registered or verified real grasp.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from grasp_place_plan import completed_source,endpoints,SOURCE_SHA,ROOT


def main():
    out=ROOT/'prepared_cube_review_20261022'
    resets=json.loads((out/'support_settling_scan.json').read_bytes())['dynamic_holds']
    source=completed_source();opened,closed=endpoints()
    before=next(s for s in source['samples'] if s['phase']=='stage_4_grasp_approach')
    n=max(20,int(np.ceil(max(abs(closed-opened))/2)))
    raw=[np.rint(opened+(closed-opened)*k/n).astype(int) for k in range(1,n+1)]
    path=[opened]*40+raw+[closed]*40
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);physics=set_cube_physics(m,side_m=.055,mass_kg=.100)
    cube=m.geom('cube').id;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_glass','support_wood')}
    bounds=m.jnt_range.copy();controls=m.actuator_ctrlrange.copy()
    gain0=m.actuator_gainprm.copy();bias0=m.actuator_biasprm.copy()
    def contacts(d):
        depth=np.zeros(3);touched=set()
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);v=max(0.,-float(c.dist)*1000)
            if cube in (a,b):
                depth[2]=max(depth[2],v);touched.add(m.geom(b if a==cube else a).name)
            elif a in support or b in support:depth[1]=max(depth[1],v)
            elif int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:depth[0]=max(depth[0],v)
        return depth,sorted(touched)
    rows=[]
    for gain in (.5,1.,1.5):
        m.actuator_gainprm[:]=gain0;m.actuator_biasprm[:]=bias0
        m.actuator_gainprm[:,0]*=gain;m.actuator_biasprm[:,1]*=gain
        for i,reset in enumerate(resets):
            d=mujoco.MjData(m);d.qpos[:]=reset['qpos']
            d.qpos[:16]=ticks_to_sim(before['actual']);d.ctrl[:]=ticks_to_sim(opened)
            mujoco.mj_forward(m,d);origin=d.qpos[-7:-4].copy()
            peak=np.zeros(3);tracking=0.;shift=0.;trace=[]
            for k,q in enumerate(path):
                goal=ticks_to_sim(q);d.ctrl[:]=goal
                for _ in range(round(.05/m.opt.timestep)):
                    mujoco.mj_step(m,d);depth,_=contacts(d);peak=np.maximum(peak,depth)
                    tracking=max(tracking,float(np.max(abs(d.qpos[:16]-goal))))
                    shift=max(shift,float(np.linalg.norm(d.qpos[-7:-4]-origin)*1000))
                if k%10==0 or k==len(path)-1:trace.append(dict(time_s=(k+1)*.05,qpos=d.qpos.tolist()))
            _,touched=contacts(d)
            row=dict(reset=i,offset_palm_mm=reset['offset_palm_mm'],gain_scale=gain,
                max_self_mm=float(peak[0]),max_support_mm=float(peak[1]),max_cube_mm=float(peak[2]),
                max_tracking_rad=tracking,max_cube_shift_mm=shift,
                final_cube_contacts=touched,finite=bool(np.isfinite(d.qpos).all()),trace=trace)
            rows.append(row);print(json.dumps({k:v for k,v in row.items() if k!='trace'}),flush=True)
    unchanged=np.array_equal(bounds,m.jnt_range) and np.array_equal(controls,m.actuator_ctrlrange)
    passed=bool(unchanged and all(r['finite'] and r['max_self_mm']<=1e-6
        and r['max_support_mm']<=.15 and r['max_tracking_rad']<=.12 for r in rows))
    summary=dict(status='offline_loaded_approach_observation_screen',
        source_empty_trial_sha256=SOURCE_SHA,approach_samples=n,approach_nominal_seconds=n*.05,
        cube_physics=physics,model_manifest=manifest,native_ranges_unchanged=bool(unchanged),
        clearance_tracking_screen_passed=passed,physical_grasp_verified=False,physical_flip_verified=False,
        marker_registration_verified=False,live_sac=False,hardware_commands_sent=0,
        initial_hand_from_saved_measurement=True,cube_poses_are_provisional_synthetic_resets=True,
        max_tracking_rad=max(r['max_tracking_rad'] for r in rows),
        max_self_mm=max(r['max_self_mm'] for r in rows),max_support_mm=max(r['max_support_mm'] for r in rows),
        max_cube_shift_mm=max(r['max_cube_shift_mm'] for r in rows),
        limitations=['No motor-current model or force sensor validates grasp force.',
                     'Cables/compliant shell contact are not fully modeled.',
                     'Real cube placement and marker-to-hand registration remain unverified.',
                     'This test does not establish force closure, retention through rotation, or policy success.'],
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),trials=rows)
    (out/'loaded_approach_sensitivity.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('trials','model_manifest')},indent=2),flush=True)
    return 0 if passed else 2

if __name__=='__main__':raise SystemExit(main())
