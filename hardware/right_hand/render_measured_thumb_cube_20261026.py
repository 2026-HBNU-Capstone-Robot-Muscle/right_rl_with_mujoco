"""Scientific model preview from saved simulation states, no hardware photos."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json
from pathlib import Path
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/measured_thumb_125659_cube_replay'
SOURCE=BASE/'trace_s40_mu0.3.npz';OUT=BASE/'preview'
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
    r=json.loads((ROOT/'thumb_clear_root_empty_20261026_125659_859144/result.json').read_bytes())
    selected=[s for s in r['samples'] if s['phase'].startswith(('stage_6_empty','stage_7_empty','stage_8a_empty','stage_9_empty','stage_10_empty','stage_11_empty','stage_2_fixed_hold'))]
    zero=selected[0]['t'];bounds={}
    for row in selected:bounds.setdefault(row['phase'],row['t']-zero)
    def stage(t):
        if t<bounds['stage_7_empty_known_root_finish']:return '기준 회전 경로'
        if t<bounds['stage_8a_empty_thumb_lift_clearance']:return '검지와 중지 추가 올림'
        if t<bounds['stage_9_empty_thumb_retreat']:return '엄지를 먼저 들어 분리'
        if t<bounds['stage_10_empty_index_middle_lift']:return '엄지를 손목 쪽으로 후퇴'
        return '검지와 중지 세움 및 유지'
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
        draw.text((10,8),'실측 관절값 재생 | 실제 큐브 회전은 미검증',font=font,fill='white')
        draw.text((10,451),label or f'{times[i]:.1f}초  {stage(times[i])}',font=small,fill=(238,215,140))
        draw.text((10,472),'큐브 위치와 마찰 및 받침 수평은 가정',font=small,fill=(185,190,198));return im
    indices=np.linspace(0,len(times)-1,96).round().astype(int);frames=[render(i) for i in indices]
    frames[0].save(OUT/'measured_path_cube.gif',save_all=True,append_images=frames[1:],duration=100,loop=0)
    split=int(np.argmin(abs(times-bounds['stage_10_empty_index_middle_lift'])))
    canvas=Image.new('RGB',(1800,490));canvas.paste(render(0,'PLACE 안내 뒤 놓을 위치 예시: 초록 위'),(0,0));canvas.paste(render(split,'엄지 후퇴 완료'),(600,0));canvas.paste(render(len(times)-1,'모델 종료: 빨강 위 / 초록 손목 방향'),(1200,0));canvas.save(OUT/'cube_placement_and_result.png');renderer.close()
    (OUT/'manifest.json').write_text(json.dumps(dict(source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        physical_rotation_verified=False,cube_initial_position_assumed=True,playback_accelerated_approximately10x=True,hardware_commands_sent=0),indent=2),encoding='utf-8')
    print('SAVED',OUT,flush=True)
if __name__=='__main__':main()
