"""Reconstruct fresh paired encoder data; export provisional-marker geometry offline."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json,hashlib,itertools
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from mujoco_playground import registry
from box_clearance import clearance
from hand_registration_geometry import candidate
from leap_pose_pair_readonly import validate_pair

root=Path(__file__).resolve().parent
source=root/'pose_pair_readonly_20261017_015118_321695/result.json'
out=root/'paired_pose_audit_20261017';out.mkdir(exist_ok=True)
data=json.loads(source.read_text(encoding='utf-8'))
assert data['status']=='photo_and_readings_saved'
check=validate_pair(*data['snapshots']);assert check['stationary_endpoints']
raw=np.array([r['position_ticks_signed'] for r in sorted(data['snapshots'][0]['motors'],key=lambda r:r['id'])])
old=json.loads((root/'backoff_video_review_20261016/readonly_193938_source.json').read_text())
oldraw=np.array([r['position_ticks_signed'] for r in sorted(old['motors'],key=lambda r:r['id'])])
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
m=registry.load('LeapCubeFaceUp',config=cfg).mj_model;d=mujoco.MjData(m)
d.qpos[:16]=raw[list(range(4,16))+list(range(4))]*2*np.pi/4096-np.pi
d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
registration=candidate();Rwm=np.array(registration['hand_to_world_candidate'])
j9=int(m.actuator_trnid[4,0]);markerworld=d.xanchor[j9]-Rwm@np.array(registration['motor_axis_landmarks_in_marker_m']['9'])
def to_marker(points):return ((np.asarray(points)-markerworld)@Rwm).tolist()
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
boxes=[];signs=np.array(list(itertools.product((-1,1),repeat=3)))
for g in range(m.ngeom):
    if int(m.geom_bodyid[g]) not in hand:continue
    if int(m.geom_type[g])==6 and (m.geom_contype[g] or m.geom_conaffinity[g]):
        vertices=d.geom_xpos[g]+(signs*m.geom_size[g])@d.geom_xmat[g].reshape(3,3).T
        boxes.append(dict(name=m.geom(g).name,body=m.body(int(m.geom_bodyid[g])).name,vertices_marker=to_marker(vertices)))
middle=[g for g in range(m.ngeom) if m.body(int(m.geom_bodyid[g])).name.startswith('mf_') and (m.geom_contype[g] or m.geom_conaffinity[g])]
thumb=[g for g in range(m.ngeom) if m.body(int(m.geom_bodyid[g])).name in ('th_px','th_ds') and (m.geom_contype[g] or m.geom_conaffinity[g])]
gaps=[]
for a,b in itertools.product(middle,thumb):
    gap,hit=clearance(d.geom_xpos[a],d.geom_xmat[a].reshape(3,3),m.geom_size[a],d.geom_xpos[b],d.geom_xmat[b].reshape(3,3),m.geom_size[b])
    gaps.append(dict(pair=[m.geom(a).name,m.geom(b).name],gap_mm=1000*gap,overlap=hit))
jointpoints={str(id):to_marker(d.xanchor[int(m.actuator_trnid[i,0])]) for i,id in enumerate(list(range(5,17))+list(range(1,5)))}
report=dict(source=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),photo=str(source.parent/'paired_photo.png'),
            raw_ticks=raw.tolist(),changes_from_old_deg=((raw-oldraw)*360/4096).tolist(),pair_check=check,
            middle_thumb_minimum=min(gaps,key=lambda g:g['gap_mm']),joint_axes_marker=jointpoints,boxes=boxes,
            provisional_registration=registration,calibration_selected=False,hardware_commands_sent=0)
(out/'geometry.json').write_text(json.dumps(report,indent=2))
for g in range(m.ngeom):
    if int(m.geom_bodyid[g]) not in hand:m.geom_group[g]=5
    for prefix,c in {'if_':[.2,.55,1,1],'mf_':[.15,.9,.25,1],'rf_':[.95,.7,.15,1],'th_':[.95,.2,.35,1]}.items():
        if m.body(int(m.geom_bodyid[g])).name.startswith(prefix):m.geom_rgba[g]=c
p=m.body('palm').id;R=d.xmat[p].reshape(3,3).copy();origin=d.xpos[p].copy()
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
canvas=Image.new('RGB',(1800,1300),'white')
for k,(label,local) in enumerate([('top',[0,0,-1]),('oblique',[.4,.4,-1]),('side A',[0,-1,-.15]),('side B',[0,1,-.15])]):
    view=R@np.array(local);view/=np.linalg.norm(view)
    up=R[:,1].copy() if k<2 else -R[:,2].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
    cam=mujoco.MjvCamera();cam.lookat[:]=origin+R@np.array([.015,-.025,-.025]);cam.distance=.40
    with mujoco.Renderer(m,height=650,width=900) as renderer:
        renderer.update_scene(d,camera=cam,scene_option=opt)
        for eye in renderer.scene.camera:eye.pos[:]=cam.lookat+view*.4;eye.forward[:]=-view;eye.up[:]=up
        im=Image.fromarray(renderer.render())
    draw=ImageDraw.Draw(im);draw.rectangle((0,0,900,54),fill='black')
    draw.text((10,8),'PAIRED 01:51 ENCODERS | '+label+' | no offsets applied',fill='white')
    draw.text((10,28),'BLUE=index | GREEN=middle | GOLD=ring | RED=thumb. Camera not registered.',fill='white')
    canvas.paste(im,((k%2)*900,(k//2)*650))
canvas.save(out/'fresh_encoder_views.jpg')
print(json.dumps({k:report[k] for k in ['raw_ticks','changes_from_old_deg','middle_thumb_minimum']},indent=2),flush=True)
