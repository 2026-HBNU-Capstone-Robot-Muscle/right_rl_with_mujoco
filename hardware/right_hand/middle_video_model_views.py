"""Render identical hypotheses from multiple camera directions, no device I/O."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from mujoco_playground import registry
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'middle_pose_review_20261017'
snap=json.loads((ROOT/'backoff_video_review_20261016/readonly_193938_source.json').read_text())
audit=json.loads((OUT/'single_joint_sensitivity.json').read_text())
raw=np.array([r['position_ticks_signed'] for r in sorted(snap['motors'],key=lambda r:r['id'])])
order=np.array(list(range(4,16))+list(range(4)));q=raw[order]*2*np.pi/4096-np.pi
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m=env.mj_model
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
for g in range(m.ngeom):
    if int(m.geom_bodyid[g]) not in hand:m.geom_group[g]=5
    for prefix,color in {'if_':[.2,.55,1,1],'mf_':[.15,.9,.25,1],'rf_':[.95,.7,.15,1],'th_':[.95,.2,.35,1]}.items():
        if m.body(int(m.geom_bodyid[g])).name.startswith(prefix):m.geom_rgba[g]=color
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
images=[];rows=[]
for hypothesis in (False,True):
    d=mujoco.MjData(m);d.qpos[:16]=q
    if hypothesis:d.qpos[4]=audit['results'][0]['best']['angle_rad']
    d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
    palm=m.body('palm').id;R=d.xmat[palm].reshape(3,3);o=d.xpos[palm]
    points={m.joint(int(m.actuator_trnid[i,0])).name:(R.T@(d.xanchor[int(m.actuator_trnid[i,0])]-o)).tolist() for i in range(4,8)}
    tip=m.geom('mf_tip').id
    points['mf_tip_center']=(R.T@(d.geom_xpos[tip]-o)).tolist()
    rows.append(dict(hypothesis_only=hypothesis,points_palm_m=points,
                fingertip_box_size=m.geom_size[tip].tolist(),fingertip_rotation_in_palm=(R.T@d.geom_xmat[tip].reshape(3,3)).tolist()))
    for side in (-1,1):
        view=R@np.array([0,side,-.12]);view/=np.linalg.norm(view)
        up=-R[:,2].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
        cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([-.02,-.025,-.05]);cam.distance=.33
        with mujoco.Renderer(m,height=650,width=850) as renderer:
            renderer.update_scene(d,camera=cam,scene_option=opt)
            for eye in renderer.scene.camera:
                eye.pos[:]=cam.lookat+view*.33;eye.forward[:]=-view;eye.up[:]=up
            im=Image.fromarray(renderer.render())
        draw=ImageDraw.Draw(im);draw.rectangle((0,0,850,54),fill='black')
        draw.text((10,8),('ID9 +51.1deg HYPOTHESIS' if hypothesis else 'BASELINE ENCODERS')+f' | side {side}',fill='white')
        draw.text((10,28),'Model camera only; NOT registered to phone camera; no calibration selected.',fill='white')
        im.save(OUT/f'side_{int(hypothesis)}_{side}.jpg');images.append(im)
canvas=Image.new('RGB',(1700,1300),'white')
for i,im in enumerate(images):canvas.paste(im,((i%2)*850,(i//2)*650))
canvas.save(OUT/'side_hypothesis_comparison.jpg')
(OUT/'side_hypothesis_geometry.json').write_text(json.dumps(rows,indent=2))
print(json.dumps(rows,indent=2))
