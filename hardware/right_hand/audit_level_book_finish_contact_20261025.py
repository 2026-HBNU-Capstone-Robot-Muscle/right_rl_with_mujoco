"""Inspect which CAD contact prevents accepting the offline continuation."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/level_book_seating'
def main():
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/'audit_scene.xml');set_cube_physics(m,.055,.1);d=mujoco.MjData(m);cube=m.geom('cube').id
    stored=np.load(OUT/'finish_s44_y15_extra128.npz');q=stored['qpos'];ts=stored['times'];peaks={}
    for t,pose in zip(ts,q):
        d.qpos[:]=pose;mujoco.mj_forward(m,d)
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            other=b if a==cube else a;n=m.geom(other).name;depth=max(0,-float(c.dist)*1000)
            if depth>peaks.get(n,{}).get('depth_mm',0):peaks[n]=dict(depth_mm=depth,t=float(t),qpos=pose.tolist(),point_world=c.pos.tolist())
    zeros=ticks_to_sim(np.zeros(16,dtype=int))
    for name,row in peaks.items():
        if row['depth_mm']<.5:continue
        pose=np.array(row['qpos']);g=m.geom(name).id;grad={}
        for mid in (1,2,3,4):
            delta=np.zeros(16);delta[mid-1]=4;dq=ticks_to_sim(delta)-zeros;distance=[]
            for sign in (-1,0,1):
                d.qpos[:]=pose;d.qpos[:16]+=sign*dq;mujoco.mj_forward(m,d);distance.append(float(mujoco.mj_geomDistance(m,d,cube,g,.1,None))*1000)
            grad[str(mid)]=dict(minus_4_ticks_mm=distance[0],unchanged_mm=distance[1],plus_4_ticks_mm=distance[2])
        row['thumb_separation_sensitivity']=grad;row['geom_solref']=m.geom_solref[g].tolist();row['geom_solimp']=m.geom_solimp[g].tolist()
    report=dict(peaks=peaks,cube_solref=m.geom_solref[cube].tolist(),cube_solimp=m.geom_solimp[cube].tolist(),not_physical_force_or_overlap_measurement=True,hardware_commands_sent=0)
    (OUT/'finish_contact_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps({n:{k:v for k,v in r.items() if k!='qpos'} for n,r in peaks.items()},indent=2),flush=True)
if __name__=='__main__':main()
