"""Render saved OFF encoders and the reached preparation, never motor commands."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from grasp_place_plan import endpoints

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024'
source=json.loads((ROOT/'bus_readonly_diagnose_20261024_043442_559037/result.json').read_bytes())
raw=np.array(source['stream_samples'][-1]['position_ticks']);opened=endpoints()[0]
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,manifest=build_with_middle_thumb_hulls(env);data=mujoco.MjData(m)
colors={'if_':[.2,.55,1,1],'mf_':[.15,.9,.25,1],'rf_':[.95,.7,.15,1],'th_':[.95,.2,.35,1]}
for g in range(m.ngeom):
    if 'cube' in (m.geom(g).name or ''):m.geom_rgba[g,3]=0;m.geom_contype[g]=m.geom_conaffinity[g]=0
    for prefix,color in colors.items():
        if m.body(m.geom_bodyid[g]).name.startswith(prefix):m.geom_rgba[g]=color
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3]=0;opt.geomgroup[4]=0
frames=[]
for label,q in (('SAVED OFF pose / DIAGNOSTIC ONLY',raw),('REACHED preparation reference',opened)):
    data.qpos[:16]=ticks_to_sim(q);data.qpos[-7:-4]=[0,0,2];data.qpos[-4:]=[1,0,0,0]
    mujoco.mj_forward(m,data);palm=m.body('palm').id;rotation=data.xmat[palm].reshape(3,3);origin=data.xpos[palm]
    for view_name,local in (('top',[0,0,-1]),('side',[0,1,-.3])):
        view=rotation@np.array(local);view/=np.linalg.norm(view)
        up=rotation[:,1];up=up-view*np.dot(up,view);up/=np.linalg.norm(up)
        camera=mujoco.MjvCamera();camera.lookat[:]=origin+rotation@np.array([.015,-.025,-.025]);camera.distance=.46
        with mujoco.Renderer(m,height=550,width=800) as renderer:
            renderer.update_scene(data,camera=camera,scene_option=opt)
            for eye in renderer.scene.camera:
                eye.pos[:]=camera.lookat+view*.46;eye.forward[:]=-view;eye.up[:]=up
            im=Image.fromarray(renderer.render())
        draw=ImageDraw.Draw(im);draw.rectangle((0,0,800,48),fill='black')
        draw.text((10,7),label+' / '+view_name,fill='white')
        draw.text((10,27),'Blue=index Green=middle Gold=ring Red=thumb | No motion approval',fill='white')
        frames.append(im)
canvas=Image.new('RGB',(1600,1100))
for i,im in enumerate(frames):canvas.paste(im,((i//2)*800,(i%2)*550))
path=OUT/'reconnected_pose_and_reference.jpg';canvas.save(path)
print(path)
