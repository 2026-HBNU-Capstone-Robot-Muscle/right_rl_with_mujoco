"""Read saved encoders and compare whole middle finger to thumb housings offline.

User reports physical middle finger touching the thumb's last motor housing.
No hypothesis in this script is installed as a calibration or hardware goal.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from mujoco_playground import registry
from box_clearance import clearance

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'backoff_video_review_20261016'
snapshot=json.loads((OUT/'readonly_193938_source.json').read_text())
trial=json.loads((ROOT/'leap_id12_backoff_empty_20261016_144922_807163.json').read_text())
order=np.array(list(range(4,16))+list(range(4)))
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg);m=env.mj_model
motors=sorted(snapshot['motors'],key=lambda r:r['id'])
raw=np.array([r['position_ticks_signed'] for r in motors])
middle=[g for g in range(m.ngeom) if m.body(int(m.geom_bodyid[g])).name.startswith('mf_')
        and (m.geom_contype[g] or m.geom_conaffinity[g])]
thumb=[g for g in range(m.ngeom) if m.body(int(m.geom_bodyid[g])).name.startswith('th_')
       and (m.geom_contype[g] or m.geom_conaffinity[g])]
assert all(int(m.geom_type[g])==6 for g in middle+thumb)
cases=[('current_OFF',raw,False),('approach_hold',trial['approach_hold']['actual_end_median'],False),
       ('return_hold',trial['return_hold']['actual_end_median'],False),
       ('current_OFF_ID10_sign_hypothesis',raw,True)]
results=[]
for name,values,flip in cases:
    d=mujoco.MjData(m);d.qpos[:16]=np.asarray(values)[order]*2*np.pi/4096-np.pi
    if flip:d.qpos[5]*=-1
    d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
    pairs=[]
    for a in middle:
        for b in thumb:
            gap,overlap=clearance(d.geom_xpos[a],d.geom_xmat[a].reshape(3,3),m.geom_size[a],
                                  d.geom_xpos[b],d.geom_xmat[b].reshape(3,3),m.geom_size[b])
            pairs.append(dict(middle_geom=m.geom(a).name,thumb_geom=m.geom(b).name,
                middle_body=m.body(int(m.geom_bodyid[a])).name,thumb_body=m.body(int(m.geom_bodyid[b])).name,
                clearance_mm=gap*1000,sat_overlap=overlap))
    by_body={body:min((p for p in pairs if p['thumb_body']==body),key=lambda p:p['clearance_mm'])
             for body in sorted({p['thumb_body'] for p in pairs})}
    row=dict(pose=name,closest_by_thumb_body=by_body,
             middle_angles_deg=np.degrees(d.qpos[4:8]).tolist(),hypothesis_only=flip)
    results.append(row)
    print(json.dumps(row),flush=True)
mapping=[]
for idx in range(4,8):
    j=int(m.actuator_trnid[idx,0]);motor=int(order[idx]+1)
    mapping.append(dict(motor_id=motor,raw_ticks=int(raw[motor-1]),joint=m.joint(j).name,
                       qpos_address=int(m.jnt_qposadr[j]),angle_deg=float((raw[motor-1]-2048)*360/4096)))
report=dict(user_observation='Only green/middle differs: real middle finger touches thumb end-motor housing.',
            earlier_thumb_similar_answer='retracted by user before viewing image',
            middle_mapping=mapping,results=results,hardware_commands_sent=0,selected_calibration=None,
            caveat='Source mapping is user-verified historically; present geometry mismatch remains. th_px and th_ds bracket the distal joint, but collision boxes are not a verified physical housing surface.')
(OUT/'middle_thumb_housing_audit.json').write_text(json.dumps(report,indent=2))
