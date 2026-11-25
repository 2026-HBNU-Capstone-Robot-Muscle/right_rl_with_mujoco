"""Static geometry illustration of the pending EMPTY segment, no devices."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np
import mujoco
from PIL import Image, ImageDraw
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
import thumb_continuation_plan as p

cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build_with_middle_thumb_hulls(env)
d=mujoco.MjData(m)
palm=m.body('palm').id;hand={palm}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
colors={'if_':[.15,.45,.95,1],'mf_':[.15,.8,.25,1],'rf_':[.95,.75,.1,1],'th_':[.95,.2,.2,1]}
for g in range(m.ngeom):
    body=m.body(int(m.geom_bodyid[g])).name
    if int(m.geom_bodyid[g]) in hand:
        m.geom_rgba[g]=colors.get(body[:3],[.4,.4,.4,1])
    else:m.geom_group[g]=5
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
start=p.source_position();rows=p.plan(start)
poses=[('Recorded OFF start (v2)',start),('Roots clearance goal',rows[p.ROOT_STEPS-1][1]),('NEXT thumb segment goal',rows[-1][1])]
canvas=Image.new('RGB',(1800,900),'white')
with mujoco.Renderer(m,height=450,width=600) as renderer:
    for col,(label,raw) in enumerate(poses):
        d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
        mujoco.mj_forward(m,d)
        rotation=d.xmat[palm].reshape(3,3);origin=d.xpos[palm]
        for row,local in enumerate(([0,0,-1],[0,-1,-.15])):
            view=rotation@np.array(local,dtype=float);view/=np.linalg.norm(view)
            up=rotation[:,1].copy() if row==0 else -rotation[:,2].copy()
            up-=view*np.dot(up,view);up/=np.linalg.norm(up)
            camera=mujoco.MjvCamera();camera.lookat[:]=origin+rotation@np.array([0,-.025,-.03]);camera.distance=.40
            renderer.update_scene(d,camera=camera,scene_option=opt)
            for eye in renderer.scene.camera:
                eye.pos[:]=camera.lookat+view*.40;eye.forward[:]=-view;eye.up[:]=up
            im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
            draw.rectangle((0,0,600,63),fill='black')
            draw.text((8,7),label+' | '+('top' if row==0 else 'side'),fill='white')
            draw.text((8,25),'Thumb RED; middle GREEN. No cube.',fill='white')
            draw.text((8,43),'STATIC geometry only; physical clearance unverified.',fill='white')
            canvas.paste(im,(col*600,row*450))
destination=p.REVIEW_PATH.parent/'thumb_continuation_views.jpg'
canvas.save(destination)
print(destination)
