"""Render saved free-cube simulation states, with explicit hypothesis labels."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,argparse
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/original_sequence_clearance'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--still-only',action='store_true');args=parser.parse_args()
    report=json.loads((OUT/'reverse_original_native_verification.json').read_bytes())
    rows=[r for r in report['runs'] if r.get('valid_start') and r['gain_scale']==1 and r['friction']==.8 and r['offset_from_nominal_mm']==[0,0]]
    moving=next(r for r in rows if r['mode']=='moving_path');control=next(r for r in rows if r['mode']=='matched_stationary_control')
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build(env,OUT/'reverse_render_CAD.xml');set_cube_physics(m,.055,.1)
    body=m.body('cube').id
    hidden_bodies={body}|set(np.flatnonzero(m.body_mocapid>=0).tolist())
    hidden_bodies.update(i for i in range(m.nbody) if any(s in m.body(i).name.lower() for s in ('goal','target')))
    m.geom_group[np.isin(m.geom_bodyid,list(hidden_bodies))]=5
    for i in range(m.ngeom):
        if any(s in m.geom(i).name.lower() for s in ('goal','target')):m.geom_group[i]=5
    option=mujoco.MjvOption();option.geomgroup[3:]=0
    data=mujoco.MjData(m);renderer=mujoco.Renderer(m,height=440,width=600)
    cam=mujoco.MjvCamera();cam.lookat[:]=[.13,-.012,-.025];cam.distance=.43;cam.elevation=-28
    R0=Rotation.from_quat(np.array(moving['initial_cube_qpos'][-4:])[[1,2,3,0]]).as_matrix()
    front=R0@np.array([-1.,0,0]);cam.azimuth=np.degrees(np.arctan2(front[1],front[0]))+180
    data.qpos[:]=moving['frames'][0];mujoco.mj_forward(m,data)
    best_view=None
    for azimuth in range(0,360,30):
        for elevation in (-25,-40,-55,-65):
            cam.azimuth=azimuth;cam.elevation=elevation;renderer.update_scene(data,camera=cam,scene_option=option)
            viewer=np.mean([camera.pos for camera in renderer.scene.camera],axis=0)-data.xpos[body]
            viewer/=np.linalg.norm(viewer)
            orange=float(viewer@(-R0[:,0]));green=float(viewer@(-R0[:,1]))
            score=orange+green if orange>.1 and green>.1 else -100.
            if best_view is None or score>best_view[0]:best_view=(score,azimuth,elevation,orange,green)
    if best_view[0]<0:raise RuntimeError('No view exposes the labeled start faces')
    _,cam.azimuth,cam.elevation,_,_=best_view
    print('VERIFIED_CAMERA_VIEW',best_view,flush=True)
    fontpath=Path('/mnt/c/Windows/Fonts/malgun.ttf')
    font=ImageFont.truetype(str(fontpath),21);small=ImageFont.truetype(str(fontpath),17)
    colors={(0,1):[.86,.035,.02,1],(0,-1):[1,.28,.015,1],(1,1):[.02,.18,.9,1],
        (1,-1):[.03,.72,.12,1],(2,1):[.94,.94,.94,1],(2,-1):[.95,.88,.025,1]}
    def render(q,verify_faces=()):
        data.qpos[:]=q;data.qvel[:]=0;mujoco.mj_forward(m,data);renderer.update_scene(data,camera=cam,scene_option=option)
        scene=renderer.scene;R=data.xmat[body].reshape(3,3);center=data.xpos[body]
        viewer=np.mean([camera.pos for camera in scene.camera],axis=0)-center
        viewer/=np.linalg.norm(viewer)
        for axis,sign in verify_faces:
            if viewer@(sign*R[:,axis])<=.1:raise RuntimeError('Labeled face is not facing the rendered camera')
        def box(size,pos,mat,rgba):
            if scene.ngeom>=scene.maxgeom:raise RuntimeError('Visualization scene full')
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom],mujoco.mjtGeom.mjGEOM_BOX,np.array(size),np.array(pos),np.array(mat).reshape(9),np.array(rgba,np.float32));scene.ngeom+=1
        box([.02735]*3,center,R,[.95,.95,.95,1])
        cell=.055/3
        for axis in range(3):
            for sign in (-1,1):
                normal=sign*R[:,axis];u=R[:,(axis+1)%3];v=np.cross(normal,u);mat=np.column_stack([u,v,normal])
                for a in (-1,0,1):
                    for b in (-1,0,1):
                        box([cell*.44,cell*.44,.00012],center+normal*.0275+u*a*cell+v*b*cell,mat,colors[axis,sign])
        return Image.fromarray(renderer.render())
    still=Image.new('RGB',(1200,530),(24,28,33));draw=ImageDraw.Draw(still)
    draw.text((14,10),'조건부 시뮬레이션 결과 | 실물 미검증',font=font,fill='white')
    still.paste(render(moving['frames'][0],((0,-1),(1,-1))),(0,50));still.paste(render(moving['frames'][-1],((0,1),(1,-1))),(600,50))
    draw=ImageDraw.Draw(still);draw.text((14,492),'시작: 초록 위 / 주황 손목 쪽',font=small,fill='white')
    draw.text((614,492),f"회전 뒤: 빨강 위 / 초록 손목 쪽 (목표 오차 {moving['final_target_error_deg']:.1f}도)",font=small,fill='white')
    still.save(OUT/'reverse_turn_start_and_finish.png')
    if args.still_only:renderer.close();return
    animation=[];indices=list(range(0,len(moving['frames']),2))
    if indices[-1]!=len(moving['frames'])-1:indices.append(len(moving['frames'])-1)
    for n,i in enumerate(indices):
        panel=Image.new('RGB',(1200,545),(24,28,33));draw=ImageDraw.Draw(panel)
        draw.text((14,8),'모델 계산 | 4배속 재생 | 실물 회전은 아직 미검증',font=font,fill='white')
        draw.text((14,43),'수정한 원본 역순 경로',font=small,fill=(110,230,160))
        draw.text((614,43),'비교: 초기 손 자세 유지',font=small,fill=(235,210,120))
        for side,row in enumerate((moving,control)):
            panel.paste(render(row['frames'][min(i,len(row['frames'])-1)]),(600*side,70))
        draw=ImageDraw.Draw(panel)
        draw.text((14,515),'목표: 빨강 위 / 초록 손목 쪽. 회전 뒤 4초 유지.',font=small,fill='white')
        animation.append(panel)
        if n%40==0:print('RENDER_REVERSE',n,len(indices),flush=True)
    animation[0].save(OUT/'reverse_turn_native_comparison.gif',save_all=True,append_images=animation[1:],duration=100,loop=0,optimize=True)
    renderer.close()
    (OUT/'reverse_visualization_manifest.json').write_text(json.dumps(dict(source='reverse_original_native_verification.json',
        geometry_for_rendering_only=True,cube_face_colors_from_documented_board_axes=True,playback_speed=4,
        physical_ninety_degree_rotation_verified=False,hardware_commands_sent=0),indent=2),encoding='utf-8')
    print('RENDER_SAVED',OUT/'reverse_turn_native_comparison.gif',flush=True)

if __name__=='__main__':main()
