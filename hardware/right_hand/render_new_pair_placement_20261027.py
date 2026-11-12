"""Offline MuJoCo illustration, not registration or a physical success claim."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco,numpy as np
from PIL import Image,ImageDraw,ImageFont
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from new_pair_grasp_plan_20261027 import ROOT,OUT,REQUEST,SEED,OPEN_PATH,NewPairGraspPlan,HOLD
def main():
    out=OUT/'guide';out.mkdir(exist_ok=True);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,out/'illustration.xml');set_cube_physics(m,.055,.1);opened=np.load(OPEN_PATH)['open_goal'];rows=NewPairGraspPlan(REQUEST).plan(NewPairGraspPlan(REQUEST).source_position());closed=next(q for phase,q in rows if phase==HOLD);review=json.loads((OUT/'review.json').read_bytes());nominal=next(r for r in review['loaded_physics'] if r['book_front_mm']==-10 and r['gain_scale']==1 and r['friction']==.3)
    d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(opened);d.qpos[-7:]=nominal['settled_cube_qpos'];mujoco.mj_forward(m,d);center=d.qpos[-7:-4].copy();cube=m.body('cube').id
    # The training asset's texture colours do not encode this user's cube.
    # Render-only slabs show the agreed two orientation cues; no model used
    # for planning, collision checks or forces is changed by this figure.
    m.geom_rgba[m.geom_bodyid==cube,3]=0
    for i in range(m.ngeom):
        name=((m.geom(i).name or '')+' '+(m.body(int(m.geom_bodyid[i])).name or '')).lower()
        if 'goal' in name or 'target' in name:m.geom_rgba[i,3]=0
    rotation=d.xmat[cube].reshape(3,3);normals=np.vstack([np.eye(3),-np.eye(3)]);wrist=d.xpos[m.body('palm').id]-center;orange=int(np.argmax((normals@rotation.T)@wrist));green=4
    renderer=mujoco.Renderer(m,height=650,width=780);opt=mujoco.MjvOption();opt.geomgroup[3:]=0;views=[]
    for az,el in ((100,-65),(15,-25),(210,-25)):
        camera=mujoco.MjvCamera();camera.lookat[:]=center+[-.025,0,-.02];camera.distance=.42;camera.azimuth=az;camera.elevation=el;renderer.update_scene(d,camera=camera,scene_option=opt)
        for j,n in enumerate(normals):
            u=np.array([1.,0,0]) if n[0]==0 else np.array([0.,1,0]);v=np.cross(n,u);mat=rotation@np.column_stack([u,v,n]);colour=[.08,.75,.17,1] if j==green else [1,.32,.02,1] if j==orange else [.55,.59,.63,1];scene=renderer.scene;mujoco.mjv_initGeom(scene.geoms[scene.ngeom],mujoco.mjtGeom.mjGEOM_BOX,np.array([.0269,.0269,.0002]),center+rotation@n*.0277,mat.ravel(),np.array(colour));scene.ngeom+=1
        views.append(Image.fromarray(renderer.render().copy()))
    renderer.close();fontpath='/mnt/c/Windows/Fonts/malgun.ttf';font=ImageFont.truetype(fontpath,29);small=ImageFont.truetype(fontpath,23);big=ImageFont.truetype(fontpath,36);canvas=Image.new('RGB',(2340,880),'#f3f5f7');draw=ImageDraw.Draw(canvas);draw.text((25,18),'PLACE CUBE 안내: 손을 연 뒤에만 놓기',font=big,fill='#15283b')
    for i,(view,label) in enumerate(zip(views,['위에서 본 모델','비스듬히 본 모델','반대쪽에서 본 모델'])):
        canvas.paste(view,(i*780,110));draw.text((i*780+20,77),label,font=font,fill='#15283b')
    draw.text((25,770),'초록 면은 위, 주황 면은 손목(ID41) 쪽. 엄지와 중지 사이의 손바닥/뿌리 위에 자연스럽게 지지.',font=font,fill='#15283b')
    draw.text((25,821),'배치 가정의 예시. 실물 좌표 측정 결과가 아님. 손 없이 안정적으로 놓이지 않으면 Q로 종료.',font=small,fill='#42566a');canvas.save(out/'placement_guide.png');print(str(out/'placement_guide.png'),flush=True)
if __name__=='__main__':main()
