"""Offline sensitivity audit, archived source bank plus latest rejected pose."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from hand_motor_mapping import ticks_to_sim
from offline_housing_proxy_model import build_with_middle_thumb_hulls
root=Path(__file__).resolve().parent;out=root/'right_hand_root_drift_20261020'
refs=[('right_hand_start_20261019/source_readonly.json','faa0c6227b8f14dc68a3cd8d928837f6f72709c7bb303b8a34e67b53917b74d8'),
      ('right_hand_start_20261019_1811/source_preflight.json','4fbb1da231f0e1361654c2f4cf83b3ceb2c25cd451ef8756699699dd9aadce70'),
      ('right_hand_start_20261019_thumb_shift/source_preflight.json','a0274a7b108f9a866c7ae680cfe321b41b3f65f5b1f0cd3f9d03303cf2531350')]
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build_with_middle_thumb_hulls(env)
palm=m.body('palm').id;hand={palm}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
supports={m.geom(n).id for n in ('support_glass','support_wood')}
d=mujoco.MjData(m);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
def check(a):
    d.qpos[:16]=ticks_to_sim(a);mujoco.mj_forward(m,d)
    support=own=0.
    for c in d.contact:
        if c.dist>=-1e-5:continue
        a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
        if ha and hb:own=max(own,float(-c.dist*1000))
        if (ha and b in supports) or (hb and a in supports):support=max(support,float(-c.dist*1000))
    return support,own
def raw_record(b):
    v=json.loads(b)
    return np.array(v['preflight_state']['position'] if 'preflight_state' in v else [x['position_ticks_signed'] for x in v['motors']])
rng=np.random.default_rng(20261020);starts=[]
for name,sha in refs:
    b=(root/name).read_bytes();assert hashlib.sha256(b).hexdigest()==sha
    raw=raw_record(b)
    offsets=[np.zeros(16,dtype=int),np.full(16,-32),np.full(16,32)]
    for i in range(16):
        for sign in (-1,1):
            delta=np.zeros(16,dtype=int);delta[i]=sign*32;offsets.append(delta)
    offsets.extend(rng.integers(-32,33,(32,16)))
    starts.extend((name,raw+delta) for delta in offsets)
fresh=(out/'source_preflight.json').read_bytes()
starts.append(('latest_rejected_pose',raw_record(fresh)))
rows=[];idx=np.array([7,11,15])
for k,(source,start) in enumerate(starts):
    assert np.all(start[idx]<2472)
    depths=[];previous=start.copy();max_step=0
    for step in range(121):
        a=start.copy();a[idx]=np.rint(start[idx]+(2472-start[idx])*step/120).astype(int)
        max_step=max(max_step,int(np.max(abs(a-previous))));previous=a
        depths.append(check(a))
    dep=np.array(depths)
    rows.append(dict(source=source,start_ticks=start.tolist(),max_step_ticks=max_step,
        max_self_overlap_mm=float(dep[:,1].max()),initial_support_overlap_mm=float(dep[0,0]),
        final_support_overlap_mm=float(dep[-1,0]),max_support_depth_increase_mm=float(max(0,np.diff(dep[:,0]).max()))))
    if (k+1)%67==0:print(f'Checked {k+1}/{len(starts)} paths',flush=True)
summary=dict(kind='kinematic_sensitivity_not_physical_certification',source_tolerance_candidate_ticks=32,
    sources=[dict(path=p,sha256=h) for p,h in refs],latest_source_sha256=hashlib.sha256(fresh).hexdigest(),
    trials=len(rows),max_self_overlap_mm=max(x['max_self_overlap_mm'] for x in rows),
    max_final_support_overlap_mm=max(x['final_support_overlap_mm'] for x in rows),
    max_support_depth_increase_mm=max(x['max_support_depth_increase_mm'] for x in rows),
    max_command_step_ticks=max(x['max_step_ticks'] for x in rows),motor_commands_sent=0,
    limitations=['Kinematic target placements, not servo dynamics or measured physical clearance.',
                 'Each reference: nominal, both all-joint extremes, 32 single-axis extremes, 32 random samples.',
                 'Sampling is not exhaustive certification of a 16-dimensional region.',
                 'Fixed thumb outside native range remains unresolved; no thumb motion or limit edits.'])
(out/'screen.json').write_text(json.dumps(dict(summary=summary,rows=rows,model=manifest),indent=2),encoding='utf-8')
print(json.dumps(summary,indent=2),flush=True)
