"""Counterfactual mounting-zero sensitivity; never installs a calibration.

One pose cannot determine offsets. These renderings are diagnostic hypotheses.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from mujoco_playground import registry
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'backoff_video_review_20261016'
hw=json.loads((OUT/'readonly_193938_source.json').read_text());ORDER=np.array(list(range(4,16))+list(range(4)))
raw=np.array([r['position_ticks_signed'] for r in sorted(hw['motors'],key=lambda r:r['id'])]);q=raw[ORDER]*2*np.pi/4096-np.pi
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m=env.mj_model
palm=m.body('palm').id;hand={palm}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
for g in range(m.ngeom):
    if int(m.geom_bodyid[g]) not in hand:m.geom_group[g]=5;m.geom_contype[g]=m.geom_conaffinity[g]=0
    for prefix,color in {'if_':[.2,.55,1,1],'mf_':[.15,.9,.25,1],'rf_':[.95,.7,.15,1],'th_':[.95,.2,.35,1]}.items():
        if m.body(m.geom_bodyid[g]).name.startswith(prefix):m.geom_rgba[g]=color
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3]=0;opt.geomgroup[4]=0;opt.geomgroup[5]=0
cases=[(None,0)]+[(i,deg) for i in range(12,16) for deg in (-90,90,180)]
rows=[];frames=[]
for index,deg in cases:
    d=mujoco.MjData(m);d.qpos[:16]=q;d.qpos[-4:]=[1,0,0,0]
    if index is not None:d.qpos[index]+=np.radians(deg)
    mujoco.mj_forward(m,d)
    distances={p:float(1000*mujoco.mj_geomDistance(m,d,m.geom('th_tip').id,m.geom(p).id,1,np.zeros(6))) for p in ('if_tip','mf_tip','rf_tip')}
    collisions=[]
    for c in d.contact:
        a,b=(m.body(m.geom_bodyid[g]).name for g in (c.geom1,c.geom2))
        if c.dist<0 and (a.startswith('th_') or b.startswith('th_')):
            collisions.append(dict(a=a,b=b,depth_mm=-float(c.dist)*1000))
    label='BASELINE' if index is None else f'HYPOTHESIS ID{index-11} angle + {deg}deg'
    row=dict(label=label,sim_joint=None if index is None else m.joint(int(m.actuator_trnid[index,0])).name,
             thumb_tip_distance_mm=distances,max_thumb_overlap_mm=max([x['depth_mm'] for x in collisions],default=0),
             changed_joint_inside_original_range=True if index is None else bool(m.jnt_range[index,0]<=d.qpos[index]<=m.jnt_range[index,1]))
    rows.append(row);print(json.dumps(row),flush=True)
    R=d.xmat[palm].reshape(3,3);o=d.xpos[palm];view=R@np.array([-.25,-.25,-1]);view/=np.linalg.norm(view)
    up=R[:,1].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
    cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([.015,-.025,-.025]);cam.distance=.44
    with mujoco.Renderer(m,height=420,width=560) as renderer:
        renderer.update_scene(d,camera=cam,scene_option=opt)
        for eye in renderer.scene.camera:eye.pos[:]=cam.lookat+view*.44;eye.forward[:]=-view;eye.up[:]=up
        im=Image.fromarray(renderer.render())
    draw=ImageDraw.Draw(im);draw.rectangle((0,0,560,45),fill='black');draw.text((8,5),label,fill='white')
    draw.text((8,23),'NOT calibrated; NO hardware changes',fill='white');frames.append(im)
canvas=Image.new('RGB',(560*4,420*4),'white')
for i,im in enumerate(frames):canvas.paste(im,((i%4)*560,(i//4)*420))
canvas.save(OUT/'thumb_zero_hypotheses.jpg')
(OUT/'thumb_zero_sensitivity.json').write_text(json.dumps(dict(results=rows,selected_calibration=None,hardware_commands_sent=0,
    caveat='Synthetic 90-degree hypotheses only. Contact distance and one oblique photo cannot establish true zero. Original joint ranges not redefined.'),indent=2))
