"""Scientific placement illustration from the offline candidate, no devices."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from PIL import Image,ImageDraw,ImageFont
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/gapped_inverse_cube'
def main():
    z=np.load(OUT/'candidate.npz')
    cubeq=json.loads((OUT/'cube_seed.json').read_bytes())['cube_qpos']
    up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build(env,OUT/'illustration_scene.xml',book_projection_mm=-10.);set_cube_physics(m,.055,.1)
    colors=dict(th=(.85,.15,.13,1.),if_=(.1,.3,.85,1.),mf=(.15,.6,.22,1.),rf=(.9,.65,.08,1.))
    for i in range(m.ngeom):
        name=m.body(int(m.geom_bodyid[i])).name or ''
        key='if_' if name.startswith('if_') else name[:2]
        if key in colors and int(m.geom_group[i])==2:
            m.geom_rgba[i]=colors[key];m.geom_matid[i]=-1
        if int(m.geom_bodyid[i])==m.body('cube').id:
            m.geom_group[i]=5
    d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(z['start']);d.qpos[-7:]=cubeq;mujoco.mj_forward(m,d)
    opt=mujoco.MjvOption();opt.geomgroup[3:]=0
    renderer=mujoco.Renderer(m,height=600,width=800)
    font=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',22)
    small=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',17)
    look=np.array(cubeq[:3]);pictures=[]
    # Use the actual modeled gravity direction as vertical in all views.
    horizontal=np.array([up[2],0.,-up[0]])
    lateral=np.array([0.,1.,0.])
    views=[('위에서 본 배치: 초록 면이 위',up*.95-horizontal*.12+lateral*.1),
           ('손목 쪽에서 본 배치: 흰 면이 손목 쪽',-horizontal*.8+up*.5+lateral*.3),
           ('옆에서 본 배치: 큐브 바닥과 받침',lateral*.8+up*.45-horizontal*.2)]
    for title,direction in views:
        direction=direction/np.linalg.norm(direction)
        cam=mujoco.MjvCamera();cam.lookat[:]=look;cam.distance=.34;cam.azimuth=120;cam.elevation=-30
        renderer.update_scene(d,camera=cam,scene_option=opt)
        # Use the user's cube face convention rather than the unrelated asset texture.
        # These decorative tiles do not modify collision geometry or model physics.
        cube_R=d.xmat[m.body('cube').id].reshape(3,3)
        faces=[([0,-1,0],[.05,.68,.15,1]),([0,1,0],[.05,.2,.88,1]),
            ([0,0,1],[.95,.95,.93,1]),([0,0,-1],[.95,.83,.06,1]),
            ([-1,0,0],[1.,.38,.02,1]),([1,0,0],[.88,.05,.04,1])]
        for normal,rgba in faces:
            normal=np.array(normal,dtype=float)
            reference=np.array([0.,0.,1.]) if abs(normal[2])<.9 else np.array([1.,0.,0.])
            u=np.cross(reference,normal);u/=np.linalg.norm(u);v=np.cross(normal,u)
            frame=cube_R@np.column_stack([u,v,normal])
            for x in (-1,0,1):
              for y in (-1,0,1):
                position=look+cube_R@(normal*.0275+u*x*.018333333333+v*y*.018333333333)
                geom=renderer.scene.geoms[renderer.scene.ngeom]
                mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_BOX,
                    np.array([.0087,.0087,.00015]),position,frame.ravel(),np.array(rgba,dtype=np.float32))
                if hasattr(geom,'emission'):geom.emission=.65
                renderer.scene.ngeom+=1
        forward=-direction
        right=np.cross(forward,up);right/=np.linalg.norm(right)
        camera_up=np.cross(right,forward)
        for camera in renderer.scene.camera:
            camera.pos[:]=look+direction*.34;camera.forward[:]=forward;camera.up[:]=camera_up
        picture=Image.fromarray(renderer.render()).convert('RGB')
        strip=Image.new('RGB',(800,650),'white');strip.paste(picture,(0,40))
        draw=ImageDraw.Draw(strip);draw.text((15,8),title,fill='black',font=font)
        draw.rectangle((0,620,800,650),fill='white')
        draw.text((15,622),'손가락: 빨강=엄지  파랑=검지  초록=중지  노랑=약지',fill='black',font=small)
        pictures.append(strip)
    renderer.close()
    canvas=Image.new('RGB',(1600,1300),'white')
    canvas.paste(pictures[0],(0,0));canvas.paste(pictures[1],(800,0));canvas.paste(pictures[2],(0,650))
    draw=ImageDraw.Draw(canvas)
    for i,line in enumerate(('역회전 시험용 큐브 배치',
        '초록 면은 위, 흰 면은 손목 ID41 쪽.',
        '엄지와 중지 사이, 손바닥 위 빈 공간에 둡니다.',
        '큐브를 놓은 뒤 작은 틈을 천천히 닫는 순서입니다.',
        '이 그림은 모델의 목표 자세입니다.',
        '실물 간격과 회전량은 별도로 확인해야 합니다.')):
        draw.text((825,700+i*45),line,fill='black',font=font)
    path=OUT/'placement_views.png';canvas.save(path)
    print(path,flush=True)

if __name__=='__main__':main()
