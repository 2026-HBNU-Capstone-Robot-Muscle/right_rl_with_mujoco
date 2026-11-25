"""Render saved physics states, clearly labelled MODEL ONLY. No device access."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json, subprocess
from pathlib import Path
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
from simulate_handoff_yaw_continuation_20261028 import OUT

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--continuous',action='store_true');args=parser.parse_args()
    source=OUT/('whole_pitch_handoff_yaw_hypothesis' if args.continuous else 'baseline_and_height_sensitivity')
    name='historical_camera_pitch_start' if args.continuous else 'retained_image_raw_orientation_up12'
    out=OUT/('continuous_visual_review' if args.continuous else 'visual_review');out.mkdir(exist_ok=True)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    m,_=build(registry.load('LeapCubeFaceUp',config=cfg),out/'render_scene.xml',book_projection_mm=-10.)
    set_cube_physics(m,.055,.1);d=mujoco.MjData(m);body=m.body('cube').id
    m.geom_rgba[m.geom_bodyid==body,3]=0
    for i in range(m.ngeom):
        label=((m.geom(i).name or '')+' '+(m.body(int(m.geom_bodyid[i])).name or '')).lower()
        if 'goal' in label or 'target' in label:m.geom_rgba[i,3]=0
    options=mujoco.MjvOption();options.geomgroup[3:]=0
    camera=mujoco.MjvCamera();camera.lookat[:]=[.105,-.035,-.02];camera.distance=.36;camera.azimuth=210;camera.elevation=-45
    renderer=mujoco.Renderer(m,height=600,width=900)
    font=ImageFont.truetype('/mnt/c/Windows/Fonts/arial.ttf',19)
    small=ImageFont.truetype('/mnt/c/Windows/Fonts/arial.ttf',16)
    data=np.load(source/(name+'_trace.npz'));states=data['qpos']
    frames=out/'frames';frames.mkdir(exist_ok=True)
    normals=np.vstack([np.eye(3),-np.eye(3)])
    colours=([.85,.04,.03,1],[.03,.15,.85,1],[.95,.95,.95,1],[1,.32,.02,1],[.08,.75,.17,1],[.98,.82,.04,1])
    for i,q in enumerate(states):
        d.qpos[:]=q;mujoco.mj_forward(m,d);renderer.update_scene(d,camera=camera,scene_option=options)
        R=d.xmat[body].reshape(3,3);C=d.qpos[-7:-4]
        for n,colour in zip(normals,colours):
            u=np.array([1.,0,0]) if n[0]==0 else np.array([0.,1,0]);v=np.cross(n,u)
            mat=R@np.column_stack([u,v,n]);scene=renderer.scene
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom],mujoco.mjtGeom.mjGEOM_BOX,
                np.array([.0269,.0269,.0002]),C+R@n*.0277,mat.ravel(),np.asarray(colour));scene.ngeom+=1
        canvas=Image.new('RGB',(900,670),(245,245,245));canvas.paste(Image.fromarray(renderer.render()),(0,0))
        draw=ImageDraw.Draw(canvas)
        draw.text((12,606),f'MODEL ONLY / saved-physics replay / 3x / simulation t={i*.3:.1f}s',font=font,fill=(15,25,35))
        note=('Nominal example; only3/27 parameter cases complete. Real linked motion unverified.' if args.continuous else
              'Unregistered image hypothesis. Other tested placements fail. NOT a robot execution.')
        draw.text((12,638),note,font=small,fill=(130,25,20))
        canvas.save(frames/f'frame_{i:05d}.jpg',quality=88)
        if i in (0,180,310,340,len(states)-1):canvas.save(out/f'view_{i:04d}.jpg',quality=92)
    renderer.close()
    video=out/('model_only_pitch_then_slow_yaw_3x.mp4' if args.continuous else 'model_only_image_hypothesis_3x.mp4')
    subprocess.run(['/usr/bin/ffmpeg','-hide_banner','-loglevel','error','-y','-framerate','10','-i',str(frames/'frame_%05d.jpg'),
        '-c:v','libx264','-preset','veryfast','-crf','22','-threads','2','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],check=True)
    # Geometry at the exact saved failed states identifies which contact needs work.
    failures=[]
    for path in (OUT/'image_pose_sensitivity9').glob('*_trace.npz'):
        if not path.stem.startswith('image_'):continue
        d.qpos[:]=np.load(path)['end_qpos'];mujoco.mj_forward(m,d);contacts=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if m.geom('cube').id in (a,b) and c.dist<0:
                other=b if a==m.geom('cube').id else a
                contacts.append(dict(other=m.geom(other).name,depth_mm=-float(c.dist)*1000))
        if contacts:failures.append(dict(trace=path.name,contacts=sorted(contacts,key=lambda c:-c['depth_mm'])[:5]))
    (out/'render_review.json').write_text(json.dumps(dict(source_trace=str(source/(name+'_trace.npz')),
        video=str(video),frames=len(states),playback_speed=3,rendered_from_saved_physics=True,
        render_only_cube_face_colours=True,post_integration_saved_state_contacts=failures,
        these_snapshot_contacts_are_not_the_maximum_failure_contact_state=True,
        original_physics_or_recordings_modified=False,hardware_execution_allowed=False,devices_opened=False),indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(video=str(video),frames=len(states),snapshot_contacts=failures)),flush=True)

if __name__=='__main__':main()
