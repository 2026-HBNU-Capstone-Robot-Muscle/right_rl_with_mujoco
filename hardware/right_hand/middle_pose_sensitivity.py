"""Single-joint geometric sensitivity, offline only; never exports motor targets.

Find whether changing one middle-finger angle within the MODEL range can
reproduce the reported thumb-housing contact. A fit is not a calibration.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import json
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from mujoco_playground import registry
from box_clearance import clearance

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'middle_pose_review_20261017';OUT.mkdir(exist_ok=True)
snap=json.loads((ROOT/'backoff_video_review_20261016/readonly_193938_source.json').read_text())
raw=np.array([r['position_ticks_signed'] for r in sorted(snap['motors'],key=lambda r:r['id'])])
order=np.array(list(range(4,16))+list(range(4)))
q0=raw[order]*2*np.pi/4096-np.pi
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg);m=env.mj_model
mid=[g for g in range(m.ngeom) if m.body(int(m.geom_bodyid[g])).name.startswith('mf_') and (m.geom_contype[g] or m.geom_conaffinity[g])]
thumb=[g for g in range(m.ngeom) if m.body(int(m.geom_bodyid[g])).name in ('th_px','th_ds') and (m.geom_contype[g] or m.geom_conaffinity[g])]
assert all(int(m.geom_type[g])==6 for g in mid+thumb)
d=mujoco.MjData(m)

def evaluate(q):
    d.qpos[:16]=q;d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
    mujoco.mj_forward(m,d)
    pairs=[]
    for a in mid:
        for b in thumb:
            gap,hit=clearance(d.geom_xpos[a],d.geom_xmat[a].reshape(3,3),m.geom_size[a],
                              d.geom_xpos[b],d.geom_xmat[b].reshape(3,3),m.geom_size[b])
            pairs.append((gap*1000,hit,m.geom(a).name,m.geom(b).name))
    gap,hit,a,b=min(pairs,key=lambda x:x[0])
    return dict(clearance_mm=gap,overlap=hit,pair=[a,b])

baseline=evaluate(q0);rows=[];render_cases=[('BASELINE: saved encoders',q0.copy(),baseline)]
for i in range(4,8):
    jid=int(m.actuator_trnid[i,0]);lo,hi=m.jnt_range[jid]
    values=np.unique(np.r_[np.linspace(lo,hi,121),q0[i]])
    trials=[]
    for val in values:
        q=q0.copy();q[i]=val
        trials.append(dict(angle_rad=float(val),delta_deg=float(np.degrees(val-q0[i])),**evaluate(q)))
    contacts=[r for r in trials if r['overlap']]
    best=min(contacts,key=lambda r:abs(r['delta_deg'])) if contacts else min(trials,key=lambda r:r['clearance_mm'])
    row=dict(motor_id=int(order[i]+1),joint=m.joint(jid).name,current_deg=float(np.degrees(q0[i])),
             found_overlap=bool(contacts),best=best,samples=trials)
    rows.append(row)
    q=q0.copy();q[i]=best['angle_rad']
    render_cases.append((f'HYPOTHESIS ID{order[i]+1}: {best["delta_deg"]:+.1f}deg',q,best))
    print(json.dumps({k:v for k,v in row.items() if k!='samples'}),flush=True)

hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
for g in range(m.ngeom):
    if int(m.geom_bodyid[g]) not in hand:m.geom_group[g]=5
    for prefix,color in {'if_':[.2,.55,1,1],'mf_':[.15,.9,.25,1],'rf_':[.95,.7,.15,1],'th_':[.95,.2,.35,1]}.items():
        if m.body(int(m.geom_bodyid[g])).name.startswith(prefix):m.geom_rgba[g]=color
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
frames=[]
for label,q,result in render_cases:
    evaluate(q)
    palm=m.body('palm').id;R=d.xmat[palm].reshape(3,3);o=d.xpos[palm]
    view=R@np.array([.25,.25,-1]);view/=np.linalg.norm(view)
    up=R[:,1].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
    cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([.015,-.025,-.025]);cam.distance=.36
    with mujoco.Renderer(m,height=600,width=800) as renderer:
        renderer.update_scene(d,camera=cam,scene_option=opt)
        for eye in renderer.scene.camera:
            eye.pos[:]=cam.lookat+view*.36;eye.forward[:]=-view;eye.up[:]=up
        im=Image.fromarray(renderer.render())
    draw=ImageDraw.Draw(im);draw.rectangle((0,0,800,62),fill='black')
    draw.text((10,8),label,fill='white')
    draw.text((10,26),f'MODEL gap {result["clearance_mm"]:.1f}mm | only GREEN middle changed',fill='white')
    draw.text((10,44),'Hypothesis only. Not a calibration. No hardware targets exported.',fill='white')
    frames.append(im)
canvas=Image.new('RGB',(1600,1800),'white')
for i,im in enumerate(frames):canvas.paste(im,((i%2)*800,(i//2)*600))
canvas.save(OUT/'single_joint_comparison.jpg')
focus=Image.new('RGB',(1600,600),'white')
focus.paste(frames[0],(0,0));focus.paste(frames[1],(800,0))
focus.save(OUT/'middle_root_comparison.jpg')
report=dict(baseline=baseline,source_timestamp=snap['timestamp'],results=rows,
            selected_calibration=None,hardware_commands_sent=0,
            caveat='One reported contact does not identify a unique joint offset; model range does not certify physical travel.')
(OUT/'single_joint_sensitivity.json').write_text(json.dumps(report,indent=2))
