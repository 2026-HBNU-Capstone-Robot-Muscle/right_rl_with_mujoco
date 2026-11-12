"""Render computed states, not edited hardware photos or claimed real success."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454'
SOURCE=BASE/'thumb_sequential_mu03/accepted_s40_000.npz'
OUT=BASE/'preview_sequential'
def main():
    OUT.mkdir(exist_ok=True);saved=np.load(SOURCE);poses=saved['qpos'];times=saved['times']
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build(env,OUT/'scene.xml');d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    palm=m.body('palm').id;origin=d.xpos[palm].copy();R=d.xmat[palm].reshape(3,3);up=-R[:,2]
    if up[2]<0:up=-up
    x=R[:,0].copy();x-=up*(x@up);x/=np.linalg.norm(x);frame=np.column_stack([x,np.cross(up,x),up]);body=m.body('cube').id
    hidden={body}|set(np.flatnonzero(m.body_mocapid>=0).tolist())
    hidden.update(i for i in range(m.nbody) if any(n in m.body(i).name.lower() for n in ('goal','target')))
    m.geom_group[np.isin(m.geom_bodyid,list(hidden))]=5
    for i in range(m.ngeom):
        if m.geom(i).name=='floor':m.geom_group[i]=5
    option=mujoco.MjvOption();option.geomgroup[3:]=0
    renderer=mujoco.Renderer(m,height=400,width=600);cam=mujoco.MjvCamera();cam.distance=.43
    font=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',17);small=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',14)
    colors={(0,1):[.86,.035,.02,1],(0,-1):[1,.28,.015,1],(1,1):[.02,.18,.9,1],(1,-1):[.03,.72,.12,1],(2,1):[.94,.94,.94,1],(2,-1):[.95,.88,.025,1]}
    def render(i,label=None):
        d.qpos[:]=poses[i];d.qvel[:]=0;mujoco.mj_forward(m,d);renderer.update_scene(d,camera=cam,scene_option=option);scene=renderer.scene
        position=origin+frame@np.array([-.22,.34,.19]);look=origin+frame@np.array([.005,.015,.055]);forward=(look-position)/np.linalg.norm(look-position);right=np.cross(forward,up);right/=np.linalg.norm(right);camera_up=np.cross(right,forward)
        for c in scene.camera:c.pos[:]=position;c.forward[:]=forward;c.up[:]=camera_up
        def box(size,pos,mat,rgba):
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom],mujoco.mjtGeom.mjGEOM_BOX,np.array(size),np.array(pos),np.array(mat).reshape(9),np.array(rgba,np.float32));scene.ngeom+=1
        A=d.xmat[body].reshape(3,3);center=d.xpos[body];box([.02735]*3,center,A,[.95,.95,.95,1]);cell=.055/3
        for axis in range(3):
            for sign in (-1,1):
                normal=sign*A[:,axis];u=A[:,(axis+1)%3];v=np.cross(normal,u);mat=np.column_stack([u,v,normal])
                for aa in (-1,0,1):
                    for bb in (-1,0,1):box([cell*.44,cell*.44,.00012],center+normal*.0275+u*aa*cell+v*bb*cell,mat,colors[axis,sign])
        im=Image.new('RGB',(600,490),(24,28,33));im.paste(Image.fromarray(renderer.render()),(0,45));draw=ImageDraw.Draw(im)
        draw.text((10,8),'계산 결과 미리보기 | 실물 성공 미검증',font=font,fill='white')
        stage=label or ('엄지를 먼저 들어 분리' if times[i]<4.8 else '엄지를 손목 쪽으로 후퇴' if times[i]<14.4 else '검지·중지 뿌리 추가 세움' if times[i]<28.8 else '빨강 위 / 초록 손목 방향 유지 관찰')
        draw.text((10,451),f'{times[i]:.1f}초  {stage}',font=small,fill=(238,215,140));draw.text((10,472),'큐브 위치와 마찰 및 받침 수평은 가정',font=small,fill=(185,190,198));return im
    split=int(np.argmin(abs(times-14.4)));indices=np.linspace(0,len(times)-1,91).round().astype(int);frames=[render(i) for i in indices]
    frames[0].save(OUT/'thumb_retreat_then_root_lift.gif',save_all=True,append_images=frames[1:],duration=100,loop=0)
    canvas=Image.new('RGB',(1800,490));canvas.paste(render(0,'기준 경로 종료: 기울어진 큐브'),(0,0));canvas.paste(render(split,'엄지 후퇴 완료'),(600,0));canvas.paste(render(len(times)-1,'검지·중지 세움과 유지 완료'),(1200,0));canvas.save(OUT/'thumb_root_comparison.png');renderer.close()
    manifest=dict(source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                  nominal_added_motion_s=28.8,final_hold_s=3,preview_playback_accelerated_approximately_3x=True,
                  physical_rotation_verified=False,hardware_commands_sent=0,initial_cube_position_and_friction_hypothetical=True)
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8');print('SAVED',OUT,flush=True)
if __name__=='__main__':main()
