"""Render actual saved simulated states and replay the short body-grip suffix."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,struct,zlib,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from research_rotation_body_hold_20261104 import CP,OUT,restore
from hand_motor_mapping import ticks_to_sim

def png(path,rgb):
    def chunk(k,b):return struct.pack('>I',len(b))+k+b+struct.pack('>I',zlib.crc32(k+b)&0xffffffff)
    h,w,_=rgb.shape;raw=b''.join(b'\0'+r.tobytes() for r in rgb)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))

def main():
    O=OUT/'views';O.mkdir(exist_ok=True);F=O/'frames';F.mkdir(exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));m.vis.global_.offwidth=1000;m.vis.global_.offheight=800
    for g in range(m.ngeom):
        name=m.body(int(m.geom_bodyid[g])).name
        color=next((v for k,v in {'if_':[.25,.47,.8,1],'mf_':[.3,.65,.42,1],'rf_':[.9,.75,.25,1],'th_':[.8,.36,.36,1]}.items() if name.startswith(k)),None)
        if color is not None:m.geom_matid[g]=-1;m.geom_rgba[g]=color
        if name=='cube':m.geom_matid[g]=-1;m.geom_rgba[g]=[.85,.85,.85,1]
    renderer=mujoco.Renderer(m,height=576,width=768);cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.lookat[:]=[.14,-.025,-.015];cam.distance=.31;cam.elevation=-28
    before=np.load(CP/'closest_state.npz');after=np.load(OUT/'lower_pair_m20_t16/end_state.npz');views=[]
    for label,z in [('before',before),('body_hold',after)]:
        d=restore(m,z)
        for side,az in [('front',135),('side',35)]:
            cam.azimuth=az;renderer.update_scene(d,camera=cam);name=label+'_'+side+'.png';png(O/name,renderer.render());views.append(name)
        cam.azimuth=35;renderer.update_scene(d,camera=cam)
        cube=m.geom('cube').id;B=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube];n=renderer.scene.ngeom
        A=Rotation.from_rotvec([0,np.pi/4,0]).as_matrix()
        mujoco.mjv_initGeom(renderer.scene.geoms[n],mujoco.mjtGeom.mjGEOM_BOX,np.array([.0275,.055/6,.0275]),C+B@np.array([0,-.055/3,0]),(B@A).ravel(),np.array([1.,.15,.15,.38],np.float32));renderer.scene.ngeom+=1
        name=label+'_upper_sweep_display.png';png(O/name,renderer.render());views.append(name)
    d=restore(m,before);goals=np.load(OUT/'lower_pair_m20_t16/research_suffix.npz')['goals'];cam.azimuth=35;frames=[]
    for i,q in enumerate(goals):
        d.ctrl[:]=ticks_to_sim(q)
        for _ in range(75):mujoco.mj_step(m,d)
        if i%2==0 or i==len(goals)-1:
            renderer.update_scene(d,camera=cam);name=f'frame_{len(frames):04d}.png';png(F/name,renderer.render());frames.append(dict(image=name,simulated_seconds=(i+1)*.075))
    renderer.close();report=dict(status='rendered_free_cube_body_hold_suffix',saved_states=views,frames=frames,not_physical_video=True,whole_cube90_not_demonstrated=True,nominal_suffix_seconds=len(goals)*.075,ghost_red_box_display_only_upper_layer45deg=True,source_checkpoint_sha256=hashlib.sha256((CP/'closest_state.npz').read_bytes()).hexdigest(),physics_and_hardware_unchanged=True,devices_opened=False)
    (O/'manifest.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='frames'}),flush=True)
if __name__=='__main__':main()
