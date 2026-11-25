"""Offline geometry of the completed slow run and the user's proposal."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454'
RUN=ROOT/'slow_forward_placement_20261026_045454_900706'

def main():
    r=json.loads((RUN/'result.json').read_bytes())
    samples=[v for v in r['samples'] if v['phase']=='stage_8_loaded_observation']
    raw=np.array(samples[-1]['actual'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'geometry_scene.xml')
    cube=m.geom('cube').id;m.geom_contype[cube]=0;m.geom_conaffinity[cube]=0
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    palm=m.body('palm').id;R=d.xmat[palm].reshape(3,3);up=-R[:,2]
    if up[2]<0:up=-up
    x=R[:,0].copy();x-=up*(x@up);x/=np.linalg.norm(x)
    frame=np.column_stack([x,np.cross(up,x),up]);origin=d.xpos[palm].copy()
    hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def snapshot(v):
        q=ticks_to_sim(np.asarray(v));d.qpos[:16]=q;d.qvel[:]=0;mujoco.mj_forward(m,d)
        points={n:((d.geom_xpos[m.geom(n).id]-origin)@frame*1000).tolist() for n in ('th_tip','if_tip','mf_tip')}
        depths={}
        for c in d.contact:
            if c.dist>=0:continue
            a,b=int(c.geom1),int(c.geom2)
            if ((int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand) or
                (a in support and int(m.geom_bodyid[b]) in hand) or (b in support and int(m.geom_bodyid[a]) in hand)):
                pair='|'.join(sorted((m.geom(a).name,m.geom(b).name)))
                depths[pair]=max(depths.get(pair,0.),-float(c.dist)*1000)
        return dict(ticks=np.asarray(v).tolist(),native_range_ok=bool(np.all(q>=m.jnt_range[:16,0]-1e-6) and np.all(q<=m.jnt_range[:16,1]+1e-6)),points_mm=points,overlaps_mm=depths)
    baseline=snapshot(raw);cases=[]
    for motor in (1,2,3,4,8,12):
        for change in (-128,-64,64,128,256):
            if motor in (8,12) and change<0:continue
            v=raw.copy();v[motor-1]+=change;z=snapshot(v)
            z.update(motor_id=motor,delta_ticks=change,point_delta_mm={n:(np.array(z['points_mm'][n])-baseline['points_mm'][n]).tolist() for n in baseline['points_mm']})
            cases.append(z)
    out=dict(source_run=RUN.name,source_phase='last loaded observation BEFORE removal',baseline=baseline,cases=cases,
             coordinate_axes='X toward fingertips; -X toward wrist; Z above book',
             geometry_calibrated=False,device_access=False,hardware_commands_sent=0)
    (OUT/'geometry_review.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
    print(json.dumps(out,indent=2),flush=True)

if __name__=='__main__':main()
