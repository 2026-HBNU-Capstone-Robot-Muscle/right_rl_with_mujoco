"""Render actual simulated checkpoint poses; visual colours only are changed."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json,hashlib,struct,zlib
import numpy as np,mujoco
def png(path,rgb):
 def chunk(kind,data):return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
 h,w,_=rgb.shape;raw=b''.join(b'\0'+row.tobytes() for row in rgb);path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/four_forward_review/model_views';O.mkdir(exist_ok=True)
CP=R/'right_hand_rotation_20261030/directional_contact_research/checkpoint_nominal';HELD=R/'right_hand_rotation_20261030/lower_body_gap_release/held_state.npz';m=mujoco.MjModel.from_binary_path(str(CP/'forward_checkpoint.mjb'));m.vis.global_.offwidth=1200;m.vis.global_.offheight=900
cubebodies={m.body('cube').id}
for b in range(m.nbody):
 if int(m.body_parentid[b]) in cubebodies:cubebodies.add(b)
for g in range(m.ngeom):
 if int(m.geom_bodyid[g]) in cubebodies:m.geom_matid[g]=-1;m.geom_rgba[g]=[.82,.84,.88,1]
for g in range(m.ngeom):
 name=m.body(int(m.geom_bodyid[g])).name
 color=next((v for k,v in {'if_':[.25,.47,.8,1],'mf_':[.3,.65,.42,1],'rf_':[.9,.75,.25,1],'th_':[.8,.36,.36,1]}.items() if name.startswith(k)),None)
 if color is not None:m.geom_matid[g]=-1;m.geom_rgba[g]=color
renderer=mujoco.Renderer(m,height=640,width=960);d=mujoco.MjData(m);frames=[]
for name,path in (('after_turn_before_regrasp',CP/'forward_checkpoint.npz'),('lower_body_held',HELD)):
 z=np.load(path);d.qpos[:]=z['qpos'];d.ctrl[:]=z['ctrl'];d.qvel[:]=0;mujoco.mj_forward(m,d)
 for view,azi in (('front',135),('side',35)):
  cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.lookat[:]=[.14,-.025,-.025];cam.distance=.37;cam.azimuth=azi;cam.elevation=-28;renderer.update_scene(d,camera=cam);rgb=renderer.render();p=O/f'{name}_{view}.png';png(p,rgb);frames.append(dict(name=name,view=view,path=p.relative_to(R).as_posix(),source=path.relative_to(R).as_posix(),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
renderer.close();(O/'manifest.json').write_text(json.dumps(dict(simulation_poses_not_physical_photos=True,render_colours_only_changed=True,views=frames),indent=2)+'\n',encoding='utf-8');print('Rendered',len(frames),'model views.')
