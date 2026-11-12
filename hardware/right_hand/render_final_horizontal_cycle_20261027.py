"""Exact final loaded queue, one nominal model, labelled 3x playback."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json
from pathlib import Path
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_model_policy_20261025 import correct_model
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'right_hand_rotation_20261027/yaw90_clearance_t8_early_root16'
PATH=FOLDER/'full_loaded_27/loaded_path.npz'
OUT=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v1/model_video'

def main():
    OUT.mkdir(exist_ok=True);frames=OUT/'movie_frames';frames.mkdir(exist_ok=True);z=np.load(PATH);prior=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());seed=next(v for v in prior['loaded_physics'] if (v['book_front_mm'],v['gain_scale'],v['friction'])==(-10.,1.,.3));up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world']);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';m,_=build(registry.load('LeapCubeFaceUp',config=cfg),OUT/'scene.xml');correct_model(m,np.array(prior['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=.002;m.opt.gravity[:]=-9.81*up;m.geom_friction[m.geom('cube').id,0]=.3;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(z['start']);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=seed['settled_cube_qpos']
    for _ in range(1000):mujoco.mj_step(m,d)
    body=m.body('cube').id;B=d.xmat[body].reshape(3,3).copy();turnB=None;m.geom_rgba[m.geom_bodyid==body,3]=0
    for i in range(m.ngeom):
        label=((m.geom(i).name or '')+' '+(m.body(int(m.geom_bodyid[i])).name or '')).lower()
        if 'goal' in label or 'target' in label:m.geom_rgba[i,3]=0
    renderer=mujoco.Renderer(m,height=600,width=900);cam=mujoco.MjvCamera();cam.lookat[:]=d.qpos[-7:-4]+[-.025,0,-.02];cam.distance=.38;cam.azimuth=210;cam.elevation=-45;option=mujoco.MjvOption();option.geomgroup[3:]=0;font=ImageFont.truetype('/mnt/c/Windows/Fonts/arial.ttf',18);small=ImageFont.truetype('/mnt/c/Windows/Fonts/arial.ttf',15);normals=np.vstack([np.eye(3),-np.eye(3)]);colours=([.85,.04,.03,1],[.03,.15,.85,1],[.95,.95,.95,1],[1,.32,.02,1],[.08,.75,.17,1],[.98,.82,.04,1]);steps=count=0;next_frame=0.;hold_yaw=None
    for i,(phase,q) in enumerate(zip(z['phases'],z['goals']),1):
        d.ctrl[:]=ticks_to_sim(q);target=round(i*.075/.002)
        for _ in range(target-steps):mujoco.mj_step(m,d)
        steps=target;R=d.xmat[body].reshape(3,3)
        if phase=='grasp_hold':turnB=R.copy()
        if i*.075+1e-6<next_frame:continue
        next_frame+=.3;renderer.update_scene(d,camera=cam,scene_option=option);C=d.qpos[-7:-4]
        for j,n in enumerate(normals):
            u=np.array([1.,0,0]) if n[0]==0 else np.array([0.,1,0]);v=np.cross(n,u);matrix=R@np.column_stack([u,v,n]);scene=renderer.scene;mujoco.mjv_initGeom(scene.geoms[scene.ngeom],mujoco.mjtGeom.mjGEOM_BOX,np.array([.0269,.0269,.0002]),C+R@n*.0277,matrix.ravel(),np.array(colours[j]));scene.ngeom+=1
        reference=turnB if turnB is not None else B;delta=Rotation.from_matrix(R@reference.T).as_quat();yaw=float(np.degrees(2*np.arctan2(delta[:3]@up,delta[3])));canvas=Image.new('RGB',(900,660),(245,245,245));canvas.paste(Image.fromarray(renderer.render()),(0,0));draw=ImageDraw.Draw(canvas);draw.text((12,606),f'MODEL ONLY / 3x / {phase} / yaw {yaw:.1f} deg',font=font,fill=(15,25,35));draw.text((12,637),'Exact proposed loaded controls; nominal contact model; physical rotation unverified',font=small,fill=(15,25,35));canvas.save(frames/f'frame_{count:05d}.jpg',quality=88)
        if count==0:canvas.save(OUT/'placement_model.jpg',quality=92)
        if phase=='yaw_hold':canvas.save(OUT/'rotation_hold_model.jpg',quality=92);hold_yaw=yaw
        count+=1
    renderer.close();r=dict(source=PATH.relative_to(ROOT).as_posix(),loaded_path_sha256=hashlib.sha256(PATH.read_bytes()).hexdigest(),frames=count,fps=10,playback_speed=3,nominal_yaw_before_open_deg=hold_yaw,final_yaw_after_open_deg=yaw,render_only_face_colours=True,model_only=True,physical_yaw_verified=False,devices_opened=False);(OUT/'render_review.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');print(json.dumps(r),flush=True)
if __name__=='__main__':main()
