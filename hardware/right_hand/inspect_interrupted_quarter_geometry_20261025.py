"""Classify initial saved OFF contacts for recovery planning, no devices."""
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
OUT=ROOT/'right_hand_rotation_20261024/reverse_quarter_resume_empty'

def main():
    raw=np.array(json.loads((OUT/'actual_off_source.json').read_bytes())['off_state']['position'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    rows=[]
    for front in (-15.,-10.,-5.):
        m,manifest=build(env,OUT/f'initial_front{front:g}.xml',book_projection_mm=front)
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:]=[10,10,10,1,0,0,0];mujoco.mj_forward(m,d)
        hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
        contacts=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if c.dist<0 and (ha and hb or a in support and hb or b in support and ha):
                contacts.append(dict(pair='|'.join(sorted((m.geom(a).name,m.geom(b).name))),depth_mm=-float(c.dist)*1000,kind='self' if ha and hb else 'support'))
        rows.append(dict(book_front_mm=front,contacts=contacts));print(json.dumps(rows[-1]),flush=True)
    (OUT/'initial_contact_diagnostic.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')

if __name__=='__main__':main()
