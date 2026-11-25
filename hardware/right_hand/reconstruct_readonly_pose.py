"""Reconstruct archived torque-OFF encoders, without motor/camera access."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json,hashlib
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from mujoco_playground import registry

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'backoff_video_review_20261016'
src=ROOT/'hand_readonly_check_result.json';raw_bytes=src.read_bytes();hw=json.loads(raw_bytes)
assert hw['status']=='read_complete_not_motion_approval'
assert all(r['torque_enabled']==0 and r['torque_enabled_end']==0 for r in hw['motors'])
motors=sorted(hw['motors'],key=lambda r:r['id']);assert [r['id'] for r in motors]==list(range(1,17))
raw=np.array([r['position_ticks_signed'] for r in motors]);ORDER=np.array(list(range(4,16))+list(range(4)))
past=json.loads((ROOT/'leap_id12_backoff_empty_20261016_144922_807163.json').read_text())
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m=env.mj_model
colors={'if_':[.2,.55,1,1],'mf_':[.15,.9,.25,1],'rf_':[.95,.7,.15,1],'th_':[.95,.2,.35,1]}
hand_bodies={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand_bodies:hand_bodies.add(b)
for g in range(m.ngeom):
    name=m.geom(g).name or ''
    if 'cube' in name:m.geom_rgba[g,3]=0;m.geom_contype[g]=m.geom_conaffinity[g]=0
    if int(m.geom_bodyid[g]) not in hand_bodies:m.geom_group[g]=5
    for prefix,color in colors.items():
        if m.body(m.geom_bodyid[g]).name.startswith(prefix):m.geom_rgba[g]=color
d=mujoco.MjData(m);d.qpos[:16]=raw[ORDER]*2*np.pi/4096-np.pi;d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
palm=m.body('palm').id;R=d.xmat[palm].reshape(3,3);o=d.xpos[palm]
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3]=0;opt.geomgroup[4]=0;opt.geomgroup[5]=0
frames=[]
for label,local_view in [('top',[0,0,-1]),('oblique_left',[.4,.4,-1]),('oblique_right',[-.4,-.4,-1]),('side',[0,1,-.3])]:
    view=R@np.array(local_view);view/=np.linalg.norm(view)
    up=R[:,1];up=up-view*np.dot(up,view);up/=np.linalg.norm(up)
    cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([.015,-.025,-.025]);cam.distance=.44
    with mujoco.Renderer(m,height=700,width=900) as renderer:
        renderer.update_scene(d,camera=cam,scene_option=opt)
        for eye in renderer.scene.camera:
            eye.pos[:]=cam.lookat+view*.44;eye.forward[:]=-view;eye.up[:]=up
        im=Image.fromarray(renderer.render())
    draw=ImageDraw.Draw(im);draw.rectangle((0,0,900,60),fill='black')
    draw.text((10,8),'TORQUE-OFF ENCODER RECONSTRUCTION | '+label,fill='white')
    draw.text((10,26),'BLUE=index 5-8 | GREEN=middle 9-12 | GOLD=ring 13-16 | RED=thumb 1-4',fill='white')
    draw.text((10,44),'Source '+hw['timestamp']+' | mapping remains to be physically checked',fill='white')
    im.save(OUT/f'readonly_{label}.jpg');frames.append(im)
canvas=Image.new('RGB',(1800,1400))
for i,im in enumerate(frames):canvas.paste(im,((i%2)*900,(i//2)*700))
canvas.save(OUT/'readonly_pose_views.jpg')
delta=(raw-np.asarray(past['return_hold']['actual_end_median']))*360/4096
dist={name:float(1000*mujoco.mj_geomDistance(m,d,m.geom('th_tip').id,m.geom(name).id,1,np.zeros(6))) for name in ('if_tip','mf_tip','rf_tip')}
report=dict(timestamp=hw['timestamp'],source_sha256=hashlib.sha256(raw_bytes).hexdigest(),raw_ticks_id_order=raw.tolist(),
            prior_return_to_current_angle_change_deg={str(i+1):float(v) for i,v in enumerate(delta)},
            thumb_tip_distances_mm=dist,hardware_commands_sent=0,
            caveat='Current encoders are after torque OFF. Phone photo 15:25 and read19:39 are not synchronized; correspondence assumes user has not moved hand.')
(OUT/'readonly_193938_source.json').write_bytes(raw_bytes)
(OUT/'readonly_pose_comparison.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
