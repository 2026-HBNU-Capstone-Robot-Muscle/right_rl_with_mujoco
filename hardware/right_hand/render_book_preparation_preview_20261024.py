"""Native model storyboard of the empty path, not a physical success image."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import argparse
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from book_empty_preparation_plan import BookPreparationPlan,ROOT,LIFT,TIP_CLEARANCE
from hand_motor_mapping import ticks_to_sim

parser=argparse.ArgumentParser();parser.add_argument('--fixture',default='current_attempt_fixture');args=parser.parse_args()
if __import__('pathlib').Path(args.fixture).name!=args.fixture:raise ValueError('Expected local fixture basename')
folder=ROOT/'right_hand_rotation_20261024/book20_current_review'/args.fixture
plan=BookPreparationPlan(folder/'request.json');raw=plan.source_position();rows=plan.plan(raw)
stages=[('현재 저장 자세',raw),('뿌리 관절 먼저 상승',[q for p,q in rows if p==LIFT][-1]),
        ('엄지 끝 짧은 경유 자세',[q for p,q in rows if p==TIP_CLEARANCE][-1]),('준비 자세 도착',rows[-1][1])]
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build_book(env,folder/'preview_scene.xml',book_projection_mm=-10.,book_ylo_mm=-125.,book_yhi_mm=65.,support_projection_mm=50.)
d=mujoco.MjData(m)
colors={'if_':[.2,.55,1,1],'mf_':[.15,.85,.3,1],'rf_':[.95,.7,.15,1],'th_':[.95,.2,.35,1]}
hand_bodies={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand_bodies:hand_bodies.add(b)
for g in range(m.ngeom):
    if (int(m.geom_bodyid[g]) not in hand_bodies
            and not (m.geom(g).name or '').startswith('support_')
            and int(m.geom_type[g])!=int(mujoco.mjtGeom.mjGEOM_PLANE)):
        m.geom_group[g]=5
    for prefix,color in colors.items():
        if m.body(m.geom_bodyid[g]).name.startswith(prefix):m.geom_rgba[g]=color
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
opt.sitegroup[:]=0
font=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',20)
canvas=Image.new('RGB',(1280,980),'#ffffff');draw=ImageDraw.Draw(canvas)
draw.text((18,12),'빈손 준비 경로 미리보기 | 실제 촬영 또는 회전 성공 영상이 아닙니다',font=font,fill='black')
draw.text((18,42),'빨강 엄지 / 파랑 검지 / 초록 중지 / 노랑 약지 | 책 두께 20mm, 앞끝 위치는 사진 추정',font=font,fill='black')
with mujoco.Renderer(m,height=400,width=640) as renderer:
    for i,(label,q) in enumerate(stages):
        d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
        palm=m.body('palm').id;rotation=d.xmat[palm].reshape(3,3);origin=d.xpos[palm]
        view=rotation@np.array([.1,.75,-.8]);view/=np.linalg.norm(view)
        up=-rotation[:,2];up-=view*np.dot(up,view);up/=np.linalg.norm(up)
        cam=mujoco.MjvCamera();cam.lookat[:]=origin+rotation@np.array([.02,-.025,-.01]);cam.distance=.45
        renderer.update_scene(d,camera=cam,scene_option=opt)
        for eye in renderer.scene.camera:
            eye.pos[:]=cam.lookat+view*.45;eye.forward[:]=-view;eye.up[:]=up
        x=(i%2)*640;y=80+(i//2)*450
        canvas.paste(Image.fromarray(renderer.render()),(x,y+30))
        draw.text((x+16,y+2),f'{i+1}. {label}',font=font,fill='black')
path=folder/'empty_preparation_preview.jpg';canvas.save(path,quality=92)
print(path,flush=True)
