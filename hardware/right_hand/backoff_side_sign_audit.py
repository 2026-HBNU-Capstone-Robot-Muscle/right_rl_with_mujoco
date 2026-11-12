"""Offline sensitivity audit of explicitly unverified side-joint signs.

Enumerates hypotheses only. Never selects or installs a hardware mapping.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import itertools,json,hashlib
import numpy as np
import mujoco
from mujoco_playground import registry

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'backoff_video_review_20261016'
ORDER=np.array(list(range(4,16))+list(range(4)))
source=ROOT/'leap_id12_backoff_empty_20261016_144922_807163.json'
hw=json.loads(source.read_text())
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg);m=env.mj_model
cube=m.geom('cube').id;cb=int(m.geom_bodyid[cube])
for g in range(m.ngeom):
    if m.geom_bodyid[g]==cb:
        m.geom_contype[g]=m.geom_conaffinity[g]=0;m.geom_rgba[g,3]=0

def measure(raw,signs):
    q=np.asarray(raw)[ORDER]*2*np.pi/4096-np.pi
    q[[1,5,9]]*=signs
    d=mujoco.MjData(m);d.qpos[:16]=q;d.ctrl[:]=q
    d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
    mujoco.mj_forward(m,d)
    pairs={}
    thumb=m.geom('th_tip').id
    for name in ('if_tip','mf_tip','rf_tip'):
        g=m.geom(name).id
        pairs[name]=float(mujoco.mj_geomDistance(m,d,thumb,g,1.,np.zeros(6))*1000)
    contacts=[]
    for c in d.contact:
        a,b=m.geom(c.geom1).name or '',m.geom(c.geom2).name or ''
        if c.dist<0 and any(x.startswith(('th_','if_','mf_','rf_')) for x in (a,b)):
            contacts.append(dict(a=a,b=b,depth_mm=-float(c.dist)*1000))
    return dict(thumb_to_tip_distance_mm=pairs,
                max_hand_penetration_mm=max([c['depth_mm'] for c in contacts],default=0),
                penetrating_contacts=contacts),d

rows=[]
for signs in itertools.product((1,-1),repeat=3):
    a,_=measure(hw['approach_hold']['actual_end_median'],signs)
    b,_=measure(hw['return_hold']['actual_end_median'],signs)
    row=dict(side_signs_id6_10_14=signs,before=a,after=b)
    rows.append(row)
    print(json.dumps(dict(signs=signs,before=a['thumb_to_tip_distance_mm'],after=b['thumb_to_tip_distance_mm'],
                         max_penetration_before=a['max_hand_penetration_mm'])),flush=True)
result=dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),rows=rows,
            selected_mapping=None,hardware_commands_sent=0,
            warning='Contact distance alone cannot identify wiring/signs or calibrate a hand. Hypotheses only; no live change.')
(OUT/'side_sign_audit.json').write_text(json.dumps(result,indent=2))
