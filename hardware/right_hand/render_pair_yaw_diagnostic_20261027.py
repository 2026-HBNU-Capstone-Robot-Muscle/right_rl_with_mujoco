"""Model-only video of the free-cube yaw response, three-times playback."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json
from pathlib import Path
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
from hand_motor_mapping import ticks_to_sim
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261027/reached_pair_yaw_study_v2'
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--candidate');args=parser.parse_args();candidate=(ROOT/args.candidate) if args.candidate else OUT/'lift4_dir1.npz';candidate.resolve().relative_to(ROOT);destination=candidate.parent/(candidate.stem+'_video') if args.candidate else OUT;destination.mkdir(exist_ok=True)
    state=np.load(OUT/'nominal_grasp_state.npz');goals=np.load(candidate)['goals'];cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,destination/'video_scene.xml');set_cube_physics(m,.055,.1);m.opt.timestep=.002;up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world']);m.opt.gravity[:]=-9.81*up;m.geom_friction[m.geom('cube').id,0]=.3;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);d=mujoco.MjData(m);d.qpos[:]=state['qpos'];d.qvel[:]=state['qvel'];d.ctrl[:]=state['ctrl']
    for _ in range(1000):mujoco.mj_step(m,d)
    body=m.body('cube').id;B=d.xmat[body].reshape(3,3).copy();m.geom_rgba[m.geom_bodyid==body,3]=0
    for i in range(m.ngeom):
        name=((m.geom(i).name or '')+' '+(m.body(int(m.geom_bodyid[i])).name or '')).lower()
        if 'goal' in name or 'target' in name:m.geom_rgba[i,3]=0
    renderer=mujoco.Renderer(m,height=600,width=900);camera=mujoco.MjvCamera();camera.lookat[:]=d.qpos[-7:-4]+[-.025,0,-.02];camera.distance=.38;camera.azimuth=210;camera.elevation=-45;option=mujoco.MjvOption();option.geomgroup[3:]=0;steps=0;next_frame=0.;normals=np.vstack([np.eye(3),-np.eye(3)]);frames=destination/'movie_frames';frames.mkdir(exist_ok=True);font=ImageFont.truetype('/mnt/c/Windows/Fonts/arial.ttf',19);small=ImageFont.truetype('/mnt/c/Windows/Fonts/arial.ttf',15);count=0
    for i,q in enumerate(np.vstack([goals,np.repeat(goals[-1][None,:],40,axis=0)])):
        d.ctrl[:]=ticks_to_sim(q);target=round((i+1)*.075/.002)
        for _ in range(target-steps):mujoco.mj_step(m,d)
        steps=target
        if (i+1)*.075+1e-6<next_frame:continue
        next_frame+=.3;renderer.update_scene(d,camera=camera,scene_option=option);R=d.xmat[body].reshape(3,3);C=d.qpos[-7:-4]
        for j,n in enumerate(normals):
            u=np.array([1.,0,0]) if n[0]==0 else np.array([0.,1,0]);v=np.cross(n,u);mat=R@np.column_stack([u,v,n]);colour=([.85,.04,.03,1],[.03,.15,.85,1],[.95,.95,.95,1],[1,.32,.02,1],[.08,.75,.17,1],[.98,.82,.04,1])[j];scene=renderer.scene;mujoco.mjv_initGeom(scene.geoms[scene.ngeom],mujoco.mjtGeom.mjGEOM_BOX,np.array([.0269,.0269,.0002]),C+R@n*.0277,mat.ravel(),np.array(colour));scene.ngeom+=1
        canvas=Image.new('RGB',(900,660),(245,245,245));canvas.paste(Image.fromarray(renderer.render()),(0,0));delta=Rotation.from_matrix(R@B.T).as_quat();yaw=float(np.degrees(2*np.arctan2(delta[:3]@up,delta[3])));draw=ImageDraw.Draw(canvas);draw.text((12,607),f'MODEL ONLY / 3x playback / cube yaw {yaw:.1f} deg',font=font,fill=(15,25,35));draw.text((12,637),'Uncalibrated contact model; 90 deg and physical yaw unverified',font=small,fill=(15,25,35));canvas.save(frames/f'frame_{count:05d}.jpg',quality=88);count+=1
    renderer.close();(destination/'render_review.json').write_text(json.dumps(dict(candidate=candidate.relative_to(ROOT).as_posix(),frames=count,playback_speed=3,video_fps=10,final_nominal_yaw_deg=yaw,render_only_face_colours=True,physics_unchanged=True,hardware_execution_allowed=False,devices_opened=False),indent=2)+'\n',encoding='utf-8');print(str(frames),count,flush=True)
if __name__=='__main__':main()
