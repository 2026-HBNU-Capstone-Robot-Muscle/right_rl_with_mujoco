"""Reconstruct measured before/after joints; screen model thumb-middle distances."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from mujoco_playground import registry
from sac_supported_start_env import SupportedStart
ROOT=Path(__file__).resolve().parent
ORDER=np.array(list(range(4,16))+list(range(4)));TICK=2*np.pi/4096

def main():
    source=ROOT/'leap_id12_backoff_empty_result.json';hw=json.loads(source.read_text())
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=SupportedStart(registry.load('LeapCubeFaceUp',config=cfg),diagnostic_cube_side_mm=55)
    m=env.mj_model;cube=m.geom('cube').id;m.geom_contype[cube]=m.geom_conaffinity[cube]=0
    # Joint descendants identify fingers independently of geometry naming.
    def descendants(root):
        ids={root}
        for i in range(m.nbody):
            if int(m.body_parentid[i]) in ids:ids.add(i)
        return ids
    thumbroot=int(m.jnt_bodyid[int(m.actuator_trnid[12,0])])
    midroot=int(m.jnt_bodyid[int(m.actuator_trnid[4,0])])
    tb,mb=descendants(thumbroot),descendants(midroot)
    # Restrict to collision geometry; visual meshes are not physical clearance evidence.
    tg=[i for i in range(m.ngeom) if m.geom_bodyid[i] in tb and (m.geom_contype[i] or m.geom_conaffinity[i])]
    mg=[i for i in range(m.ngeom) if m.geom_bodyid[i] in mb and (m.geom_contype[i] or m.geom_conaffinity[i])]
    base=json.loads((ROOT/'sac_cube55_resets.json').read_text())['candidates'][0]['qpos']
    rows=[];frames=[]
    opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3]=0;opt.geomgroup[4]=0
    for label,raw in [('BEFORE: measured hold3290',hw['approach_hold']['actual_end_median']),
                      ('AFTER: measured hold3130',hw['return_hold']['actual_end_median']),
                      ('BEFORE: commanded goals',hw['approach_hold']['goal'])]:
        d=mujoco.MjData(m);d.qpos[:]=base;d.qpos[:16]=np.asarray(raw)[ORDER]*TICK-np.pi
        d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
        pairs=[]
        for a in tg:
            for b in mg:
                seg=np.zeros(6);distance=mujoco.mj_geomDistance(m,d,a,b,.2,seg)
                pairs.append(dict(thumb=m.geom(a).name or f'geom{a}',middle=m.geom(b).name or f'geom{b}',
                    thumb_body=m.body(m.geom_bodyid[a]).name,middle_body=m.body(m.geom_bodyid[b]).name,
                    signed_distance_mm=float(distance*1000),nearest_points=seg.tolist()))
        pairs.sort(key=lambda p:p['signed_distance_mm'])
        contacts=[dict(a=m.geom(c.geom1).name,b=m.geom(c.geom2).name,
            body_a=m.body(m.geom_bodyid[c.geom1]).name,body_b=m.body(m.geom_bodyid[c.geom2]).name,
            penetration_mm=-float(c.dist)*1000) for c in d.contact if c.dist<0]
        row=dict(label=label,raw_id_order=raw,closest_thumb_middle=pairs[:5],penetrating_contacts=contacts)
        rows.append(row);print(json.dumps(row),flush=True)
        # Cube is absent for this diagnostic, hide its render geometry too.
        cb=m.geom_bodyid[cube]
        for i in range(m.ngeom):
            if m.geom_bodyid[i]==cb:m.geom_rgba[i,3]=0
        with mujoco.Renderer(m,height=600,width=800) as renderer:
            camera=mujoco.MjvCamera();camera.lookat[:]=[.07,0,-.02];camera.distance=.34;camera.azimuth=100;camera.elevation=-70
            renderer.update_scene(d,camera=camera,scene_option=opt)
            im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
            draw.rectangle((0,0,800,60),fill='black');draw.text((10,8),label,fill='white')
            draw.text((10,28),'MODEL reconstructed from encoders; NOT proof of real contact',fill='white')
            frames.append(im)
    report=dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_timestamp=hw['timestamp'],
        thumb_bodies=[m.body(i).name for i in sorted(tb)],middle_bodies=[m.body(i).name for i in sorted(mb)],
        results=rows,motor_commands_sent=0,physical_contact_verified=False,
        caveat='Collision proxies, mount frame and cable geometry not fully calibrated. Positive model distance does not exclude physical contact.')
    (ROOT/'sac_backoff_contact_review_result.json').write_text(json.dumps(report,indent=2)+'\n')
    canvas=Image.new('RGB',(2400,600))
    for i,im in enumerate(frames):canvas.paste(im,(i*800,0))
    canvas.save(ROOT/'sac_backoff_contact_review.jpg')

if __name__=='__main__':main()
