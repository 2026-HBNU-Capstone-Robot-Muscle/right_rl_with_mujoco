"""Render saved physics state without re-simulating or prescribing a cube pose."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json,struct,zlib,hashlib
import numpy as np,mujoco
R=Path(__file__).resolve().parent;CP=R/'right_hand_rotation_20261104/raised_cradle_160_yaw0/trace_nominal';O=R/'right_hand_rotation_20261104/model_views';O.mkdir(exist_ok=True)
def png(path,rgb):
 def chunk(k,b):return struct.pack('>I',len(b))+k+b+struct.pack('>I',zlib.crc32(k+b)&0xffffffff)
 h,w,_=rgb.shape;raw=b''.join(b'\0'+x.tobytes() for x in rgb);path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))
m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));m.vis.global_.offwidth=1000;m.vis.global_.offheight=800
for g in range(m.ngeom):
 name=m.body(int(m.geom_bodyid[g])).name;color=next((v for k,v in {'if_':[.25,.47,.8,1],'mf_':[.3,.65,.42,1],'rf_':[.9,.75,.25,1],'th_':[.8,.36,.36,1]}.items() if name.startswith(k)),None)
 if color is not None:m.geom_matid[g]=-1;m.geom_rgba[g]=color
 if name=='cube':m.geom_matid[g]=-1;m.geom_rgba[g]=[.85,.85,.85,1]
z=np.load(CP/'closest_state.npz');d=mujoco.MjData(m);d.qpos[:]=z['qpos'];d.ctrl[:]=z['ctrl'];d.qvel[:]=z['qvel'];mujoco.mj_forward(m,d);renderer=mujoco.Renderer(m,height=600,width=900);manifest=[]
for view,azi in (('front',135),('side',35)):
 cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.lookat[:]=[.14,-.025,-.025];cam.distance=.37;cam.azimuth=azi;cam.elevation=-28;renderer.update_scene(d,camera=cam);png(O/(view+'.png'),renderer.render());manifest.append(view+'.png')
renderer.close();row=json.loads((CP/'review.json').read_bytes())['cases'][0];(O/'manifest.json').write_text(json.dumps(dict(saved_simulation_state=True,not_a_physical_photo=True,measured_whole_yaw_deg=row['best_whole_yaw_deg'],sample=row['best_sample'],source_state_sha256=hashlib.sha256((CP/'closest_state.npz').read_bytes()).hexdigest(),render_colours_only_changed=True,views=manifest,motor_writes=0),indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(views=manifest,whole_yaw_deg=row['best_whole_yaw_deg'])))
