"""Review image of the support-aware settled simulation start, no hardware."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
model,_=build(env);data=mujoco.MjData(model)
reset=json.loads((ROOT/'sac_supported_reset_result.json').read_text());data.qpos[:]=reset['candidates'][0]['qpos'];mujoco.mj_forward(model,data)
cube=model.geom('cube').id;body=model.geom_bodyid[cube]
for g in range(model.ngeom):
    if model.geom_bodyid[g]==body:model.geom_rgba[g,3]=0
option=mujoco.MjvOption();option.geomgroup[2]=1;option.geomgroup[3]=0;option.geomgroup[4]=0
colors=[(1,1,1),(1,.85,0),(.9,.04,.04),(1,.38,0),(.05,.2,1),(.05,.75,.1)]
normals=np.array([[0,0,1],[0,0,-1],[1,0,0],[-1,0,0],[0,1,0],[0,-1,0]])
images=[]
with mujoco.Renderer(model,height=600,width=800) as renderer:
    for azimuth,elevation in [(100,-35),(90,-85)]:
        camera=mujoco.MjvCamera();camera.lookat[:]=[.08,0,-.025];camera.distance=.36;camera.azimuth=azimuth;camera.elevation=elevation
        renderer.update_scene(data,camera=camera,scene_option=option)
        R=data.geom_xmat[cube].reshape(3,3);center=data.geom_xpos[cube]
        for normal,color in zip(normals,colors):
            u=np.cross(np.eye(3)[np.argmin(abs(normal))],normal);v=np.cross(normal,u);mat=R@np.stack([u,v,normal],axis=1)
            scene=renderer.scene;geom=scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_BOX,np.array([.02725,.02725,.0001]),center+R@normal*.0273,mat.ravel(),np.array([*color,1],dtype=np.float32));scene.ngeom+=1
        im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im);draw.rectangle((0,0,800,62),fill='black')
        draw.text((12,10),'OFFLINE SIMULATION START | derived, not a measured hardware pose',fill='white')
        draw.text((12,29),'Wood/glass forward projection: 34 mm | glass thickness: 6 mm',fill='white')
        draw.text((12,45),'Cube colors follow the user face IDs; view direction differs from webcam',fill='white');images.append(im)
sheet=Image.new('RGB',(1600,600))
for i,im in enumerate(images):sheet.paste(im,(i*800,0))
sheet.save(ROOT/'sac_supported_start_preview.jpg')
print(ROOT/'sac_supported_start_preview.jpg')
