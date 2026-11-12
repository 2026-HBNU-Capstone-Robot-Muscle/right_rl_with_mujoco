"""Audit a large-displacement apparent turn instead of silently relaxing gates."""
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,manifest=build(env);d=mujoco.MjData(m);report=json.loads((ROOT/'sac_supported_teacher_batch_eval.json').read_text())
cube=m.geom('cube').id;floor=m.geom('floor').id;rows=[]
for row in report['results']:
    contacts=[];longest=streak=0
    for rec in row['trace'][1:]:
        d.qpos[:]=rec['qpos'];mujoco.mj_forward(m,d)
        partners=[]
        for c in d.contact:
            if cube in (c.geom1,c.geom2) and c.dist<=.001:
                g=c.geom2 if c.geom1==cube else c.geom1;partners.append(m.geom(g).name)
        orientation=(70<=rec['tip']<=110 and rec['axis']<=25 and rec['error']<=25)
        hand=[p for p in partners if p.startswith(('if_','mf_','rf_','th_','palm_'))]
        streak=streak+1 if orientation and hand and not any(p.startswith(('floor','support_')) for p in partners) else 0
        longest=max(longest,streak)
        if rec['step']%20==0 or rec is row['trace'][-1]:contacts.append(dict(step=rec['step'],tip_deg=rec['tip'],shift_mm=rec['shift'],cube_pos=rec['qpos'][-7:-4],contacts=partners))
    rows.append(dict(offset=row['offset_index'],seed=row['seed'],longest_orientation_with_hand_contact_steps=longest,samples=contacts))
result=dict(status='offline_native_contact_reconstruction_from_MJX_traces',gravity=m.opt.gravity.tolist(),floor_pos=d.geom_xpos[floor].tolist(),
            floor_normal_world=d.geom_xmat[floor].reshape(3,3)[:,2].tolist(),rows=rows,
            caveat='Native contact reconstruction is not identical to MJX contacts; it diagnoses whether apparent rotation followed a fall, not new success certification.')
(ROOT/'sac_supported_teacher_contact_audit_result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(gravity=result['gravity'],floor_pos=result['floor_pos'],floor_normal=result['floor_normal_world'],rows=[dict(offset=r['offset'],seed=r['seed'],hold=r['longest_orientation_with_hand_contact_steps'],final=r['samples'][-1]) for r in rows]),indent=2))
