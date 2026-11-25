"""Render the conditional prescribed-hand replay as a scientific artifact."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/completed_empty_path_review_20261025'
SOURCE=OUT/'measured_nominal_cube_replay_kinematic_v2.npz'

def main():
    trace=np.load(SOURCE);poses=trace['qpos'];times=trace['times'];errors=trace['goal_errors_deg']
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build(env,OUT/'measured_render.xml');set_cube_physics(m,.055,.1)
    body=m.body('cube').id;hidden={body}|set(np.flatnonzero(m.body_mocapid>=0).tolist())
    hidden.update(i for i in range(m.nbody) if any(n in m.body(i).name.lower() for n in ('goal','target')))
    m.geom_group[np.isin(m.geom_bodyid,list(hidden))]=5
    option=mujoco.MjvOption();option.geomgroup[3:]=0
    renderer=mujoco.Renderer(m,height=420,width=560);d=mujoco.MjData(m)
    old=json.loads((ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_075405_loaded_model/short_visualization_manifest.json').read_bytes())
    camera=mujoco.MjvCamera();camera.lookat[:]=[.13,-.012,-.025];camera.distance=.43;camera.azimuth=old['camera_azimuth'];camera.elevation=old['camera_elevation']
    colors={(0,1):[.86,.035,.02,1],(0,-1):[1,.28,.015,1],(1,1):[.02,.18,.9,1],(1,-1):[.03,.72,.12,1],(2,1):[.94,.94,.94,1],(2,-1):[.95,.88,.025,1]}
    font=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',17);small=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',14)
    def render(i):
        d.qpos[:]=poses[i];d.qvel[:]=0;mujoco.mj_forward(m,d);renderer.update_scene(d,camera=camera,scene_option=option)
        scene=renderer.scene;R=d.xmat[body].reshape(3,3);center=d.xpos[body]
        def box(size,pos,mat,rgba):
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom],mujoco.mjtGeom.mjGEOM_BOX,np.array(size),np.array(pos),np.array(mat).reshape(9),np.array(rgba,np.float32));scene.ngeom+=1
        box([.02735]*3,center,R,[.95,.95,.95,1]);cell=.055/3
        for axis in range(3):
            for sign in (-1,1):
                normal=sign*R[:,axis];u=R[:,(axis+1)%3];v=np.cross(normal,u);mat=np.column_stack([u,v,normal])
                for a in (-1,0,1):
                    for b in (-1,0,1):box([cell*.44,cell*.44,.00012],center+normal*.0275+u*a*cell+v*b*cell,mat,colors[axis,sign])
        im=Image.new('RGB',(560,492),(24,28,33));im.paste(Image.fromarray(renderer.render()),(0,38));draw=ImageDraw.Draw(im)
        draw.text((10,8),'실제 빈손 궤적을 사용한 가정 계산 | 실물 회전 미검증',font=font,fill='white')
        draw.text((10,464),f'{times[i]:.1f}초 / 목표 오차 {errors[i]:.1f}도 | 3배속 | 적재 응답 미검증',font=small,fill=(238,215,140))
        return im
    indices=np.linspace(0,len(poses)-1,169).round().astype(int);frames=[render(i) for i in indices]
    frames[0].save(OUT/'measured_quarter_diagnostic.gif',save_all=True,append_images=frames[1:],duration=83,loop=0)
    comparison=Image.new('RGB',(1120,492));comparison.paste(frames[0],(0,0));comparison.paste(frames[-1],(560,0));comparison.save(OUT/'measured_quarter_start_end.png')
    renderer.close()
    (OUT/'render_manifest.json').write_text(json.dumps(dict(source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        prescribed_empty_joint_trajectory_under_hypothetical_load=True,physical_cube_rotation_verified=False,actual_loaded_motor_dynamics_identified=False,
        source_actual_runs_are_separate=True,frame_count=169,nominal_speedup=3,render_only_facelets=True,hardware_commands_sent=0),indent=2),encoding='utf-8')
    print('SAVED',OUT/'measured_quarter_diagnostic.gif',flush=True)

if __name__=='__main__':main()
