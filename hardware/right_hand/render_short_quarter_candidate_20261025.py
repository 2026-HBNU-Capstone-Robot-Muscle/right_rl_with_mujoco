"""Scientific comparison from saved native states; rendering only."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_075405_loaded_model'
SOURCE=OUT/'tip_clearance_search_v3/candidate_g1_stop38_tip-24_ring-56.npz'

def main():
    trace=np.load(SOURCE);initial=trace['initial_state'];cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/'short_render.xml');set_cube_physics(m,.055,.1)
    q0=initial[1:1+m.nq];q1=trace['states'][-1,1:1+m.nq];body=m.body('cube').id
    hidden={body}|set(np.flatnonzero(m.body_mocapid>=0).tolist())
    hidden.update(i for i in range(m.nbody) if any(n in m.body(i).name.lower() for n in ('goal','target')))
    m.geom_group[np.isin(m.geom_bodyid,list(hidden))]=5
    option=mujoco.MjvOption();option.geomgroup[3:]=0
    d=mujoco.MjData(m);renderer=mujoco.Renderer(m,height=420,width=560)
    cam=mujoco.MjvCamera();cam.lookat[:]=[.13,-.012,-.025];cam.distance=.43
    rotations=[Rotation.from_quat(q[-4:][[1,2,3,0]]).as_matrix() for q in (q0,q1)]
    best=None
    for az in range(0,360,30):
      for el in (-25,-40,-55,-65):
        cam.azimuth=az;cam.elevation=el;scores=[]
        for q,R,faces in zip((q0,q1),rotations,(((0,-1),(1,-1)),((0,1),(1,-1)))):
            d.qpos[:]=q;mujoco.mj_forward(m,d);renderer.update_scene(d,camera=cam,scene_option=option)
            v=np.mean([c.pos for c in renderer.scene.camera],axis=0)-d.xpos[body];v/=np.linalg.norm(v)
            scores.extend(v@(sign*R[:,axis]) for axis,sign in faces)
        score=sum(scores) if min(scores)>.1 else -100
        if best is None or score>best[0]:best=(score,az,el)
    if best[0]<0:raise ValueError('Cannot expose stated start/end faces')
    _,cam.azimuth,cam.elevation=best
    colors={(0,1):[.86,.035,.02,1],(0,-1):[1,.28,.015,1],(1,1):[.02,.18,.9,1],(1,-1):[.03,.72,.12,1],(2,1):[.94,.94,.94,1],(2,-1):[.95,.88,.025,1]}
    def render(q):
        d.qpos[:]=q;d.qvel[:]=0;mujoco.mj_forward(m,d);renderer.update_scene(d,camera=cam,scene_option=option)
        scene=renderer.scene;R=d.xmat[body].reshape(3,3);center=d.xpos[body]
        def box(size,pos,mat,rgba):
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom],mujoco.mjtGeom.mjGEOM_BOX,np.array(size),np.array(pos),np.array(mat).reshape(9),np.array(rgba,np.float32));scene.ngeom+=1
        box([.02735]*3,center,R,[.95,.95,.95,1]);cell=.055/3
        for axis in range(3):
          for sign in (-1,1):
            normal=sign*R[:,axis];u=R[:,(axis+1)%3];v=np.cross(normal,u);mat=np.column_stack([u,v,normal])
            for a in (-1,0,1):
              for b in (-1,0,1):box([cell*.44,cell*.44,.00012],center+normal*.0275+u*a*cell+v*b*cell,mat,colors[axis,sign])
        return Image.fromarray(renderer.render())
    target=target_quaternion(q0[-4:]);error=float(np.degrees(2*np.arccos(np.clip(abs(q1[-4:]@target),0,1))))
    angle=float(np.degrees(Rotation.from_matrix(rotations[1]@rotations[0].T).magnitude()))
    f=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',20);small=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',16)
    image=Image.new('RGB',(1120,525),(24,28,33));draw=ImageDraw.Draw(image)
    draw.text((14,9),'새 회전 후보의 기준 조건 모델 결과 | 실물 회전 미검증',font=f,fill='white')
    image.paste(render(q0),(0,45));image.paste(render(q1),(560,45));draw=ImageDraw.Draw(image)
    draw.text((14,472),'시작: 초록 위 / 주황 손목 쪽',font=small,fill='white')
    draw.text((574,472),f'38초 경로 + 4초 정지: 목표 자세 오차 {error:.1f}도',font=small,fill='white')
    draw.text((14,501),'모터 응답과 마찰이 달라지면 미달하는 조건도 있음. 다음 실물 검사는 큐브 없는 전체 관절 경로.',font=small,fill=(235,210,120))
    image.save(OUT/'short_quarter_start_and_finish.png');renderer.close()
    (OUT/'short_visualization_manifest.json').write_text(json.dumps(dict(source=str(SOURCE.relative_to(ROOT)),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        camera_azimuth=cam.azimuth,camera_elevation=cam.elevation,whole_cube_rotation_deg=angle,final_target_error_deg=error,
        saved_native_physics_states_rendered=True,render_only_facelets=True,physical_cube_rotation_verified=False,hardware_commands_sent=0),indent=2),encoding='utf-8')
    print('SHORT_RENDER_SAVED',angle,error,flush=True)

if __name__=='__main__':main()
