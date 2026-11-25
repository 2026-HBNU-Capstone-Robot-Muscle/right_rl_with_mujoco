"""Render saved replay with explicit cube colors; no policy or hardware run."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,argparse
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--run');args=p.parse_args()
source=ROOT/args.run/'supported_batch_eval.json' if args.run else ROOT/'sac_supported_motion_search_20hz_result_mjx_eval.json'
out=ROOT/args.run if args.run else ROOT
report=json.loads(source.read_text())
row=next(r for r,s in zip(report['results'],report['summaries']) if s['success']);trace=row['trace']
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';model,_=build(registry.load('LeapCubeFaceUp',config=cfg));data=mujoco.MjData(model)
cube=model.geom('cube').id;body=model.geom_bodyid[cube]
for g in range(model.ngeom):
    if model.geom_bodyid[g]==body:model.geom_rgba[g,3]=0
option=mujoco.MjvOption();option.geomgroup[2]=1;option.geomgroup[3]=0;option.geomgroup[4]=0
colors=[(1,1,1),(1,.85,0),(.9,.04,.04),(1,.38,0),(.05,.2,1),(.05,.75,.1)]
normals=np.array([[0,0,1],[0,0,-1],[1,0,0],[-1,0,0],[0,1,0],[0,-1,0]])
frames=[];selection=list(range(0,len(trace),3))
if selection[-1]!=len(trace)-1:selection.append(len(trace)-1)
with mujoco.Renderer(model,height=600,width=800) as renderer:
    camera=mujoco.MjvCamera();camera.lookat[:]=[.08,0,-.015];camera.distance=.34;camera.azimuth=100;camera.elevation=-45
    for index in selection:
        rec=trace[index];data.qpos[:]=rec['qpos'];mujoco.mj_forward(model,data);renderer.update_scene(data,camera=camera,scene_option=option)
        R=data.geom_xmat[cube].reshape(3,3);center=data.geom_xpos[cube]
        for normal,color in zip(normals,colors):
            u=np.cross(np.eye(3)[np.argmin(abs(normal))],normal);v=np.cross(normal,u);mat=R@np.stack([u,v,normal],axis=1)
            scene=renderer.scene;geom=scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_BOX,np.array([.02725,.02725,.0001]),center+R@normal*.0273,mat.ravel(),np.array([*color,1],dtype=np.float32));scene.ngeom+=1
        image=Image.fromarray(renderer.render());draw=ImageDraw.Draw(image);draw.rectangle((0,0,800,65),fill='black')
        label='learned SAC correction + reference' if args.run else 'reference only, SAC correction not applied'
        draw.text((12,8),'OFFLINE MJX REPLAY | '+label,fill='white')
        draw.text((12,26),f"t={rec['step']*.05:.2f}s | desired turn={rec.get('tip',0):.1f} deg | hold={rec.get('hold',0)} steps",fill='white')
        draw.text((12,44),'34 mm support projection / 6 mm glass | modeled result, not real-hand validation',fill='white');frames.append(image)
stem='sac_corrected_flip' if args.run else 'sac_supported_reference_flip'
frames[0].save(out/(stem+'.gif'),save_all=True,append_images=frames[1:],duration=150,loop=0)
sheet=Image.new('RGB',(1600,600));sheet.paste(frames[0],(0,0));sheet.paste(frames[-1],(800,0));sheet.save(out/(stem+'_comparison.jpg'))
print('Saved reference GIF and comparison; no motor commands.')
