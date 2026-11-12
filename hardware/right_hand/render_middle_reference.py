"""Render two offline middle-finger reference poses; no motor goal export."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from mujoco_playground import registry
root=Path(__file__).resolve().parent;out=root/'paired_pose_audit_20261017'
raw=np.array(json.loads((out/'geometry.json').read_text())['raw_ticks'])
q=raw[list(range(4,16))+list(range(4))]*2*np.pi/4096-np.pi
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';m=registry.load('LeapCubeFaceUp',config=cfg).mj_model
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
for g in range(m.ngeom):
    if int(m.geom_bodyid[g]) not in hand:m.geom_group[g]=5
    m.geom_rgba[g]=[.35,.35,.4,1]
    if m.body(int(m.geom_bodyid[g])).name.startswith('mf_'):m.geom_rgba[g]=[.1,.85,.2,1]
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
canvas=Image.new('RGB',(1800,1300),'white');report=[]
for pose,root_angle in enumerate((0,np.pi/2)):
    d=mujoco.MjData(m);d.qpos[:16]=q;d.qpos[4:8]=[root_angle,0,0,0];d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
    p=m.body('palm').id;R=d.xmat[p].reshape(3,3).copy();o=d.xpos[p].copy()
    anchors=[(R.T@(d.xanchor[int(m.actuator_trnid[i,0])]-o)).tolist() for i in range(4,8)]
    report.append(dict(model_middle_angles_deg=np.degrees(d.qpos[4:8]).tolist(),anchors_palm_m=anchors))
    for side,local in enumerate(([0,0,-1],[0,-1,-.05])):
        view=R@np.array(local);view/=np.linalg.norm(view)
        up=R[:,1].copy() if side==0 else -R[:,2].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
        cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([.02,-.025,-.025]);cam.distance=.44
        with mujoco.Renderer(m,height=650,width=900) as renderer:
            renderer.update_scene(d,camera=cam,scene_option=opt)
            for eye in renderer.scene.camera:eye.pos[:]=cam.lookat+view*.44;eye.forward[:]=-view;eye.up[:]=up
            im=Image.fromarray(renderer.render())
        dr=ImageDraw.Draw(im);dr.rectangle((0,0,900,54),fill='black')
        dr.text((10,8),f'MODEL REFERENCE root {np.degrees(root_angle):.0f}deg, other middle joints 0 | '+('top' if side==0 else 'side'),fill='white')
        dr.text((10,28),'Geometry illustration only. No motor command generated.',fill='white')
        im.save(out/f'middle_reference_{pose}_{side}.jpg');canvas.paste(im,(side*900,pose*650))
canvas.save(out/'middle_reference_comparison.jpg');(out/'reference_geometry.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2),flush=True)

# A single, unambiguous user-facing reference: middle only, upright to palm.
for g in range(m.ngeom):
    body=m.body(int(m.geom_bodyid[g])).name
    if int(m.geom_bodyid[g]) in hand and body!='palm' and not body.startswith('mf_'):
        m.geom_group[g]=5
d.qpos[4:8]=[np.pi/2,0,0,0];mujoco.mj_forward(m,d)
reference=Image.new('RGB',(1800,700),'white')
for side,local in enumerate(([.1,-1,-.1],[1,0,-.2])):
    view=R@np.array(local);view/=np.linalg.norm(view)
    up=-R[:,2].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
    cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([0,-.02,-.06]);cam.distance=.35
    with mujoco.Renderer(m,height=700,width=900) as renderer:
        renderer.update_scene(d,camera=cam,scene_option=opt)
        for eye in renderer.scene.camera:eye.pos[:]=cam.lookat+view*.35;eye.forward[:]=-view;eye.up[:]=up
        im=Image.fromarray(renderer.render())
    dr=ImageDraw.Draw(im);dr.rectangle((0,0,900,70),fill='black')
    dr.text((10,8),'ONE REFERENCE POSE | middle finger straight, upright to palm',fill='white')
    dr.text((10,28),'Two views of SAME pose. Other fingers hidden for clarity, not repositioned.',fill='white')
    dr.text((10,48),'No powered move. Do not force joints. Measurement only; no automatic calibration.',fill='white')
    reference.paste(im,(side*900,0))
reference.save(out/'middle_reference_upright.jpg')
