"""Offline whole-trial geometric replay. No device access or runtime setting changes."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json,hashlib
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from mujoco_playground import registry
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'backoff_video_review_20261016'
src=ROOT/'leap_id12_backoff_empty_20261016_144922_807163.json'
hw=json.loads(src.read_text());ORDER=np.array(list(range(4,16))+list(range(4)))
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg);m=env.mj_model
cube=m.geom('cube').id;cb=int(m.geom_bodyid[cube])
COLORS={'if_':[.2,.55,1,1],'mf_':[.15,.9,.25,1],'rf_':[.95,.7,.15,1],'th_':[.95,.2,.35,1]}
for g in range(m.ngeom):
    if m.geom_bodyid[g]==cb:m.geom_contype[g]=m.geom_conaffinity[g]=0;m.geom_rgba[g,3]=0
    for prefix,color in COLORS.items():
        if m.body(m.geom_bodyid[g]).name.startswith(prefix):m.geom_rgba[g]=color
tips={p:m.geom(p+'tip').id for p in COLORS}
rows=[];phases={};frames=[]
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3]=0;opt.geomgroup[4]=0
for sample in hw['samples']:
    d=mujoco.MjData(m);d.qpos[:16]=np.asarray(sample['actual'])[ORDER]*2*np.pi/4096-np.pi
    d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
    distances={p:float(1000*mujoco.mj_geomDistance(m,d,tips['th_'],tips[p],1,np.zeros(6))) for p in ('if_','mf_','rf_')}
    row=dict(t=sample['t'],phase=sample['phase'],thumb_tip_distances_mm=distances)
    rows.append(row);phases.setdefault(row['phase'],[]).append(row)

representatives=[]
for phase,rr in phases.items():
    result=dict(phase=phase,start_s=rr[0]['t'],end_s=rr[-1]['t'],
                minimum_thumb_tip_distance_mm={p:min(r['thumb_tip_distances_mm'][p] for r in rr) for p in ('if_','mf_','rf_')})
    print(json.dumps(result),flush=True)
    representatives.append(result)
    # Select last measured sample of each phase; separate view from phone imagery.
    sample=next(s for s in reversed(hw['samples']) if s['phase']==phase)
    d=mujoco.MjData(m);d.qpos[:16]=np.asarray(sample['actual'])[ORDER]*2*np.pi/4096-np.pi
    d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
    palm=m.body('palm').id;R=d.xmat[palm].reshape(3,3);o=d.xpos[palm]
    view=-R[:,2];cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([.015,-.025,-.03]);cam.distance=.30
    cam.azimuth=float(np.degrees(np.arctan2(view[1],view[0])));cam.elevation=float(np.degrees(np.arcsin(view[2])))
    with mujoco.Renderer(m,height=600,width=800) as renderer:
        renderer.update_scene(d,camera=cam,scene_option=opt)
        # Explicit camera basis avoids azimuth/elevation convention ambiguity.
        for eye in renderer.scene.camera:
            eye.pos[:]=cam.lookat+view*.30
            eye.forward[:]=-view
            eye.up[:]=R[:,0]
        im=Image.fromarray(renderer.render())
    draw=ImageDraw.Draw(im);draw.rectangle((0,0,800,62),fill='black')
    draw.text((10,6),f'MODEL ONLY {phase} t={sample["t"]:.2f}s since torque ON',fill='white')
    draw.text((10,24),'BLUE=index 5-8 | GREEN=middle 9-12 | GOLD=ring 13-16 | RED=thumb 1-4',fill='white')
    draw.text((10,42),'Encoder reconstruction; phone video time is NOT synchronized',fill='white')
    im.save(OUT/f'model_{phase}.jpg');frames.append(im)
canvas=Image.new('RGB',(1600,600*((len(frames)+1)//2)))
for i,im in enumerate(frames):canvas.paste(im,((i%2)*800,(i//2)*600))
canvas.save(OUT/'model_phase_sheet.jpg')
(OUT/'full_timeline_model.json').write_text(json.dumps(dict(source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),phases=representatives,samples=rows,hardware_commands_sent=0,physical_contact_verified=False),indent=2))
