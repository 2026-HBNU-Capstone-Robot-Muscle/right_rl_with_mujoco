"""Offline counterexamples to unique calibration from one apparent contact.

The 20-degree pad-tilt band is an exploratory assumption, NOT a video
measurement. Samples do not authorize motion or identify encoder offsets.
"""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import itertools
import json
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image, ImageDraw
from mujoco_playground import registry
from box_clearance import clearance

root = Path(__file__).resolve().parent
out = root / 'middle_pose_review_20261017'
snap = json.loads((root/'backoff_video_review_20261016/readonly_193938_source.json').read_text())
raw = np.array([r['position_ticks_signed'] for r in sorted(snap['motors'], key=lambda r:r['id'])])
q0 = raw[list(range(4,16))+list(range(4))]*2*np.pi/4096-np.pi
cfg = registry.get_default_config('LeapCubeFaceUp'); cfg.impl = 'jax'
m = registry.load('LeapCubeFaceUp', config=cfg).mj_model
d = mujoco.MjData(m)
d.qpos[:16] = q0; d.qpos[-7:-4] = [0,0,2]; d.qpos[-4:] = [1,0,0,0]
mujoco.mj_forward(m,d)
palm = m.body('palm').id; R = d.xmat[palm].reshape(3,3).copy(); origin = d.xpos[palm].copy()
tip = m.geom('mf_tip').id
thumb = [g for g in range(m.ngeom) if m.body(int(m.geom_bodyid[g])).name in ('th_px','th_ds') and (m.geom_contype[g] or m.geom_conaffinity[g])]
assert all(m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX for g in [tip]+thumb)
thumb_data = [(g,d.geom_xpos[g].copy(),d.geom_xmat[g].reshape(3,3).copy(),m.geom_size[g].copy()) for g in thumb]
ranges = [m.jnt_range[int(m.actuator_trnid[i,0])] for i in range(4,8)]
grid = [np.linspace(*ranges[0],41), np.unique(np.clip([-q0[5],0,q0[5]],*ranges[1])), np.linspace(*ranges[2],31), np.linspace(*ranges[3],31)]
hits=[]; checked=0
for angles in itertools.product(*grid):
    checked += 1
    d.qpos[4:8] = angles; mujoco.mj_forward(m,d)
    rot = d.geom_xmat[tip].reshape(3,3); pos = d.geom_xpos[tip]
    tilt = float(np.degrees(np.arcsin(np.clip(abs(np.dot(R[:,2],rot[:,2])),0,1))))
    if tilt>20: continue
    for g,c,r,h in thumb_data:
        if np.linalg.norm(pos-c)>np.linalg.norm(m.geom_size[tip])+np.linalg.norm(h)+.002: continue
        gap,overlap=clearance(pos,rot,m.geom_size[tip],c,r,h)
        if not overlap and gap<=.002:
            hits.append(dict(angles_rad=list(angles),delta_deg=np.degrees(np.array(angles)-q0[4:8]).tolist(),
                             gap_mm=gap*1000,tilt_deg=tilt,thumb_geom=m.geom(g).name))
            break
print(f'Checked {checked}; near-contact samples {len(hits)}',flush=True)
# Select geometrically distinct examples, not best-fit calibrations.
chosen=[]
for row in sorted(hits,key=lambda r:np.linalg.norm(r['delta_deg'])):
    if all(np.linalg.norm(np.array(row['delta_deg'])-c['delta_deg'])>25 for c in chosen):
        chosen.append(row)
    if len(chosen)==3: break
report=dict(samples=checked,near_contact_samples=len(hits),examples=chosen,
            assumptions=dict(tip_tilt_band_deg=20,maximum_box_gap_mm=2,side_joint_samples_rad=grid[1].tolist()),
            selected_calibration=None,hardware_targets_exported=False,
            limitations=['Unregistered camera: tilt band is a hypothesis, not measured orientation.',
                         'Only tip-to-thumb gap checked; other self/support collisions NOT certified.',
                         'Static model-range samples, not physical range or dynamic validation.'])
(out/'multi_joint_candidates.json').write_text(json.dumps(report,indent=2))
hand={m.body('palm').id}
for body in range(m.nbody):
    if int(m.body_parentid[body]) in hand:hand.add(body)
for g in range(m.ngeom):
    if int(m.geom_bodyid[g]) not in hand:m.geom_group[g]=5
    name=m.body(int(m.geom_bodyid[g])).name
    for prefix,col in {'if_':[.2,.55,1,1],'mf_':[.15,.9,.25,1],'rf_':[.95,.7,.15,1],'th_':[.95,.2,.35,1]}.items():
        if name.startswith(prefix): m.geom_rgba[g]=col
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
canvas=Image.new('RGB',(900,650*max(1,len(chosen))),'white')
for k,row in enumerate(chosen):
    d.qpos[:16]=q0;d.qpos[4:8]=row['angles_rad'];mujoco.mj_forward(m,d)
    view=R@np.array([0,-1,-.15]);view/=np.linalg.norm(view)
    up=-R[:,2].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
    cam=mujoco.MjvCamera();cam.lookat[:]=origin+R@np.array([-.02,-.025,-.05]);cam.distance=.33
    with mujoco.Renderer(m,height=650,width=900) as renderer:
        renderer.update_scene(d,camera=cam,scene_option=opt)
        for eye in renderer.scene.camera:
            eye.pos[:]=cam.lookat+view*.33;eye.forward[:]=-view;eye.up[:]=up
        im=Image.fromarray(renderer.render())
    draw=ImageDraw.Draw(im);draw.rectangle((0,0,900,70),fill='black')
    draw.text((10,8),f'COUNTEREXAMPLE {k+1}: multiple middle joints changed; NOT a fitted pose',fill='white')
    draw.text((10,28),f'Pad tilt {row["tilt_deg"]:.1f}deg; tip-thumb box gap {row["gap_mm"]:.2f}mm',fill='white')
    draw.text((10,48),'Other collisions unchecked. No hardware targets. Camera not registered.',fill='white')
    canvas.paste(im,(0,k*650))
canvas.save(out/'multi_joint_counterexamples.jpg')
print(json.dumps(report,indent=2),flush=True)
