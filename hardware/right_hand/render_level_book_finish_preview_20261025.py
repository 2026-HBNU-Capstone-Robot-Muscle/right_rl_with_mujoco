"""Scientific preview of one explicitly hypothetical whole-path candidate."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;BASE=ROOT/'right_hand_rotation_20261024/support_photo_review_221703/level_book_seating'
SOURCE=BASE/'whole_finish128_dt0.0010_tc0.004_mu0.3/case_s44_y15.npz';OUT=BASE/'preview';OUT.mkdir(exist_ok=True)
def main():
    stored=np.load(SOURCE);poses=stored['qpos'];times=stored['times'];errors=stored['goal_errors_deg'];cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,OUT/'preview_scene.xml');set_cube_physics(m,.055,.1)
    d=mujoco.MjData(m);mujoco.mj_forward(m,d);p=m.body('palm').id;R=d.xmat[p].reshape(3,3);origin=d.xpos[p].copy();up=-R[:,2];up=up if up[2]>0 else -up;x=R[:,0].copy();x-=up*(x@up);x/=np.linalg.norm(x);frame=np.column_stack([x,np.cross(up,x),up]);body=m.body('cube').id
    hidden={body}|set(np.flatnonzero(m.body_mocapid>=0).tolist());hidden.update(i for i in range(m.nbody) if any(n in m.body(i).name.lower() for n in ('goal','target')));m.geom_group[np.isin(m.geom_bodyid,list(hidden))]=5
    for i in range(m.ngeom):
        if m.geom(i).name=='floor':m.geom_group[i]=5
    option=mujoco.MjvOption();option.geomgroup[3:]=0;renderer=mujoco.Renderer(m,height=420,width=600);cam=mujoco.MjvCamera();cam.lookat[:]=[.13,-.01,-.025];cam.distance=.43;cam.azimuth=115;cam.elevation=-25
    colors={(0,1):[.86,.035,.02,1],(0,-1):[1,.28,.015,1],(1,1):[.02,.18,.9,1],(1,-1):[.03,.72,.12,1],(2,1):[.94,.94,.94,1],(2,-1):[.95,.88,.025,1]};font=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',17);small=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',14)
    def render(i):
        d.qpos[:]=poses[i];d.qvel[:]=0;mujoco.mj_forward(m,d);renderer.update_scene(d,camera=cam,scene_option=option);scene=renderer.scene
        position=origin+frame@np.array([-.22,.34,.19]);look=origin+frame@np.array([.025,.015,.035]);forward=(look-position)/np.linalg.norm(look-position);right=np.cross(forward,up);right/=np.linalg.norm(right);camera_up=np.cross(right,forward)
        for c in scene.camera:c.pos[:]=position;c.forward[:]=forward;c.up[:]=camera_up
        A=d.xmat[body].reshape(3,3);center=d.xpos[body]
        def box(size,pos,mat,rgba):
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom],mujoco.mjtGeom.mjGEOM_BOX,np.array(size),np.array(pos),np.array(mat).reshape(9),np.array(rgba,np.float32));scene.ngeom+=1
        box([.02735]*3,center,A,[.95,.95,.95,1]);cell=.055/3
        for axis in range(3):
            for sign in (-1,1):
                normal=sign*A[:,axis];u=A[:,(axis+1)%3];v=np.cross(normal,u);mat=np.column_stack([u,v,normal])
                for aa in (-1,0,1):
                    for bb in (-1,0,1):box([cell*.44,cell*.44,.00012],center+normal*.0275+u*aa*cell+v*bb*cell,mat,colors[axis,sign])
        im=Image.new('RGB',(600,504),(24,28,33));im.paste(Image.fromarray(renderer.render()),(0,45));draw=ImageDraw.Draw(im);draw.text((10,9),'수평 받침 가정의 시뮬레이션 | 실물 회전 미검증',font=font,fill='white')
        stage='기존 기록 경로' if times[i]<=times[-1]-6.2+.01 else '새 마무리와 정지 관찰';draw.text((10,472),f'{times[i]:.1f}초 / {stage} / 목표 오차 {errors[i]:.1f}도',font=small,fill=(238,215,140));return im
    before=int(np.argmin(abs(times-(times[-1]-6.2))));indices=np.r_[np.linspace(0,before,73).round().astype(int),np.linspace(before,len(times)-1,76).round().astype(int)];frames=[render(i) for i in indices]
    frames[0].save(OUT/'level_book_finish_candidate.gif',save_all=True,append_images=frames[1:],duration=[65]*73+[83]*76,loop=0)
    compare=Image.new('RGB',(1800,504));compare.paste(render(0),(0,0));compare.paste(render(before),(600,0));compare.paste(render(len(times)-1),(1200,0));compare.save(OUT/'start_old_end_new_end.png');renderer.close()
    (OUT/'manifest.json').write_text(json.dumps(dict(source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),level_book_assumption=True,hypothetical_contact_time=.004,cube_friction=.3,hand_motion_prescribed_not_actuator_identified=True,physical_cube_rotation_verified=False,nonuniform_preview_timing=True,hardware_commands_sent=0),indent=2),encoding='utf-8');print('SAVED',OUT,flush=True)
if __name__=='__main__':main()
