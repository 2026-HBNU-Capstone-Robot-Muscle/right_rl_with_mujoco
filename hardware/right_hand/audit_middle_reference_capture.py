"""Offline comparison only: no controller imports, motor commands or calibration install."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image, ImageDraw
from mujoco_playground import registry
from box_clearance import clearance
from leap_pose_pair_readonly import validate_pair
from hand_registration_geometry import candidate

ROOT = Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source',default='pose_pair_readonly_20261017_061429_504276/result.json')
parser.add_argument('--out',default='middle_reference_review_20261017')
args=parser.parse_args()
OUT = ROOT / args.out
OUT.mkdir(exist_ok=True)
sources = [ROOT/args.source,
           ROOT/'pose_pair_readonly_20261017_015118_321695/result.json']
records = [json.loads(p.read_text(encoding='utf-8')) for p in sources]
for r in records:
    assert r['status'] == 'photo_and_readings_saved'
    assert validate_pair(*r['snapshots'])['stationary_endpoints']
raw = [np.array([x['position_ticks_signed'] for x in sorted(r['snapshots'][0]['motors'], key=lambda x:x['id'])]) for r in records]
qs = [x[list(range(4,16))+list(range(4))]*2*np.pi/4096-np.pi for x in raw]
nominal = np.array([90.,0.,0.,0.])
measured = np.degrees(qs[0][4:8])
hypotheses = [('Unmodified mapping', np.zeros(4)),
              ('Quarter-turn offset hypothesis', np.array([90.,0.,0.,-90.])),
              ('Nominal-reference fit (not calibration)', nominal-measured)]
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
m=registry.load('LeapCubeFaceUp',config=cfg).mj_model
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
for g in range(m.ngeom):
    if int(m.geom_bodyid[g]) not in hand:m.geom_group[g]=5
    for prefix,color in {'if_':[.2,.55,1,1], 'mf_':[.1,.85,.2,1], 'rf_':[.95,.7,.15,1], 'th_':[.95,.2,.35,1]}.items():
        if m.body(int(m.geom_bodyid[g])).name.startswith(prefix):m.geom_rgba[g]=color
boxes=[g for g in range(m.ngeom) if int(m.geom_type[g])==6 and (m.geom_contype[g] or m.geom_conaffinity[g])]
middle=[g for g in boxes if m.body(int(m.geom_bodyid[g])).name.startswith('mf_')]
thumb=[g for g in boxes if m.body(int(m.geom_bodyid[g])).name in ('th_px','th_ds') and m.geom(g).name!='th_tip']
def gap(d,aa,bb):
    rows=[]
    for a,b in itertools.product(aa,bb):
        dist,hit=clearance(d.geom_xpos[a],d.geom_xmat[a].reshape(3,3),m.geom_size[a],d.geom_xpos[b],d.geom_xmat[b].reshape(3,3),m.geom_size[b])
        rows.append(dict(pair=[m.geom(a).name,m.geom(b).name],gap_mm=float(dist*1000),overlap=bool(hit)))
    return min(rows,key=lambda x:x['gap_mm'])
report=dict(sources=[dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),pair_check=r['pair_check']) for p,r in zip(sources,records)],
            measured_reference_middle_deg=measured.tolist(),nominal_reference_middle_deg=nominal.tolist(),
            old_middle_deg=np.degrees(qs[1][4:8]).tolist(),
            all_motor_change_deg=((raw[0]-raw[1])*360/4096).tolist(),
            calibration_selected=False,hardware_commands_sent=0,
            limitations=['Reference angles are an approximate manual pose, not metrology.',
                         'Single pose cannot identify rotation signs or distinguish zero offsets from assembly-frame differences.',
                         'Model views are not registered to the camera.',
                         'Collision at imposed static coordinates does not establish a safe motion.'],cases=[])
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
canvas=Image.new('RGB',(2100,1200),'white')
for row,q in enumerate(qs):
    for col,(label,offset) in enumerate(hypotheses):
        d=mujoco.MjData(m);d.qpos[:16]=q;d.qpos[4:8]+=np.radians(offset)
        d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
        tip=[g for g in middle if m.geom(g).name=='mf_tip']
        contacts=[]
        for i in range(d.ncon):
            c=d.contact[i];a,b=int(c.geom1),int(c.geom2)
            if a in middle or b in middle:
                contacts.append(dict(pair=[m.geom(a).name,m.geom(b).name],distance_mm=float(c.dist*1000)))
        case=dict(pose='reference_'+sources[0].parent.name.split('_')[-2][:4] if row==0 else 'previous_contact_0151',hypothesis=label,
                  middle_offset_deg=offset.tolist(),middle_angles_deg=np.degrees(d.qpos[4:8]).tolist(),
                  minimum_middle_thumb=gap(d,middle,thumb),tip_thumb=gap(d,tip,thumb),
                  middle_native_contacts=contacts,
                  middle_outside_model_limits=[m.joint(int(m.actuator_trnid[i,0])).name for i in range(4,8)
                      if not m.jnt_range[int(m.actuator_trnid[i,0]),0] <= d.qpos[i] <= m.jnt_range[int(m.actuator_trnid[i,0]),1]])
        report['cases'].append(case)
        if row==0:
            registration=candidate();Rwm=np.array(registration['hand_to_world_candidate'])
            markerworld=d.xanchor[int(m.actuator_trnid[4,0])]-Rwm@np.array(registration['motor_axis_landmarks_in_marker_m']['9'])
            def marker(pts):return ((np.asarray(pts)-markerworld)@Rwm).tolist()
            ids=list(range(5,17))+list(range(1,5))
            case['joints_marker_m']={str(ident):marker(d.xanchor[int(m.actuator_trnid[i,0])]) for i,ident in enumerate(ids)}
            corners=np.array(list(itertools.product((-1,1),repeat=3)))
            case['middle_boxes_marker_m']=[dict(name=m.geom(g).name,vertices=marker(d.geom_xpos[g]+(corners*m.geom_size[g])@d.geom_xmat[g].reshape(3,3).T)) for g in middle]
            case['registration_verified']=False
        p=m.body('palm').id;R=d.xmat[p].reshape(3,3);o=d.xpos[p]
        view=R@np.array([.25,-1,-.25]);view/=np.linalg.norm(view)
        up=-R[:,2].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
        cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([0,-.02,-.04]);cam.distance=.40
        with mujoco.Renderer(m,height=600,width=700) as renderer:
            renderer.update_scene(d,camera=cam,scene_option=opt)
            for eye in renderer.scene.camera:eye.pos[:]=cam.lookat+view*.4;eye.forward[:]=-view;eye.up[:]=up
            im=Image.fromarray(renderer.render())
        draw=ImageDraw.Draw(im);draw.rectangle((0,0,700,80),fill='black')
        draw.text((10,8),case['pose']+' | '+label,fill='white')
        draw.text((10,26),'Middle angles: '+', '.join(f'{x:.1f}' for x in case['middle_angles_deg'])+' deg',fill='white')
        draw.text((10,44),f"Tip-thumb box gap: {case['tip_thumb']['gap_mm']:.1f} mm | green=middle red=thumb",fill='white')
        draw.text((10,62),'OFFLINE HYPOTHESIS ONLY | camera not registered | no hardware changes',fill='white')
        canvas.paste(im,(col*700,row*600))
canvas.save(OUT/'two_pose_comparison.jpg')
# Exhaust signs against the second pose; do not choose a sign from one reference.
report['sign_checks']=[]
for signs in itertools.product((-1,1),repeat=4):
    d=mujoco.MjData(m);d.qpos[:16]=qs[1]
    d.qpos[4:8]=np.radians(nominal)+np.array(signs)*(qs[1][4:8]-qs[0][4:8])
    d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
    inside=all(m.jnt_range[int(m.actuator_trnid[i,0]),0] <= d.qpos[i] <= m.jnt_range[int(m.actuator_trnid[i,0]),1] for i in range(4,8))
    report['sign_checks'].append(dict(signs=list(signs),old_middle_deg=np.degrees(d.qpos[4:8]).tolist(),
                                     within_model_limits=bool(inside),tip_thumb=gap(d,tip,thumb)))
(OUT/'analysis.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(dict(measured_reference_middle_deg=report['measured_reference_middle_deg'],
                      cases=len(report['cases']),sign_checks=len(report['sign_checks']),
                      sign_checks_within_model_limits=sum(x['within_model_limits'] for x in report['sign_checks']),
                      output=str(OUT)),indent=2))
