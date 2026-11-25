"""Saved-image/encoder hypotheses only. No bus, motion or calibration writes."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import hashlib, itertools, json
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image, ImageDraw
from mujoco_playground import registry
from box_clearance import clearance
from hand_registration_geometry import candidate
from leap_pose_pair_readonly import validate_pair

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'middle_direction_review_20261018'
OUT.mkdir(exist_ok=True)
sources=[('baseline','pose_pair_readonly_20261018_072023_698689/baseline/result.json'),
         ('bent','pose_pair_readonly_20261018_072023_698689/bent/result.json'),
         ('reference_0326','pose_pair_readonly_20261018_032605_137975/result.json'),
         ('contact_0151','pose_pair_readonly_20261017_015118_321695/result.json')]
records=[]
for label,name in sources:
    p=ROOT/name;r=json.loads(p.read_text(encoding='utf-8'))
    check=validate_pair(*r['snapshots']);assert check['stationary_endpoints']
    raw=np.mean([[x['position_ticks_signed'] for x in sorted(s['motors'],key=lambda x:x['id'])] for s in r['snapshots']],axis=0)
    records.append(dict(label=label,path=name,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),raw_ticks=raw.tolist(),pair_check=check))
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
m=registry.load('LeapCubeFaceUp',config=cfg).mj_model
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
for g in range(m.ngeom):
    if int(m.geom_bodyid[g]) not in hand:m.geom_group[g]=5
    for prefix,color in {'if_':[.2,.55,1,1],'mf_':[.1,.85,.2,1],'rf_':[.95,.7,.15,1],'th_':[.95,.2,.35,1]}.items():
        if m.body(int(m.geom_bodyid[g])).name.startswith(prefix):m.geom_rgba[g]=color
boxes=[g for g in range(m.ngeom) if int(m.geom_type[g])==6 and (m.geom_contype[g] or m.geom_conaffinity[g])]
middle=[g for g in boxes if m.body(int(m.geom_bodyid[g])).name.startswith('mf_')]
thumb=[g for g in boxes if m.body(int(m.geom_bodyid[g])).name in ('th_px','th_ds') and m.geom(g).name!='th_tip']
tip=m.geom('mf_tip').id
registration=candidate();Rwm=np.array(registration['hand_to_world_candidate'])
corners=np.array(list(itertools.product((-1,1),repeat=3)))
def state(raw,order):
    ids=list(range(5,9))+list(order)+list(range(13,17))+list(range(1,5))
    d=mujoco.MjData(m);d.qpos[:16]=np.asarray(raw)[np.asarray(ids)-1]*2*np.pi/4096-np.pi
    d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
    mujoco.mj_forward(m,d)
    return d,ids
def metrics(d,ids):
    gaps=[]
    for b in thumb:
        gap,overlap=clearance(d.geom_xpos[tip],d.geom_xmat[tip].reshape(3,3),m.geom_size[tip],d.geom_xpos[b],d.geom_xmat[b].reshape(3,3),m.geom_size[b])
        gaps.append(dict(other=m.geom(b).name,gap_mm=float(gap*1000),overlap=bool(overlap)))
    markerworld=d.xanchor[int(m.actuator_trnid[4,0])]-Rwm@np.array(registration['motor_axis_landmarks_in_marker_m']['9'])
    def marker(pts):return ((np.asarray(pts)-markerworld)@Rwm).tolist()
    return dict(middle_angles_deg=np.degrees(d.qpos[4:8]).tolist(),
        outside_limits=[m.joint(int(m.actuator_trnid[i,0])).name for i in range(4,8) if not m.jnt_range[int(m.actuator_trnid[i,0]),0] <= d.qpos[i] <= m.jnt_range[int(m.actuator_trnid[i,0]),1]],
        tip_thumb=min(gaps,key=lambda x:x['gap_mm']),
        tip_world_m=d.geom_xpos[tip].tolist(),tip_rotation_world=d.geom_xmat[tip].reshape(3,3).tolist(),
        joints_marker_m={str(ident):marker(d.xanchor[int(m.actuator_trnid[i,0])]) for i,ident in enumerate(ids)},
        middle_boxes_marker_m=[dict(name=m.geom(g).name,vertices=marker(d.geom_xpos[g]+(corners*m.geom_size[g])@d.geom_xmat[g].reshape(3,3).T)) for g in middle],
        registration_verified=False)
report=dict(status='offline_motor_order_hypotheses',sources=records,mapping_verified=False,calibration_selected=False,hardware_commands_sent=0,
    assumptions=['Common encoder conversion raw * 2pi/4096 - pi; no fitted offsets.',
                 'Only middle permutation varied; other three fingers held at existing mapping.',
                 'Old contact photo and new motion are independent constraints; contact is not an optimization target.',
                 'Marker-to-palm transform is provisional. Pixel overlays cannot certify mapping alone.',
                 'No commanded motion or SAC result is produced by this audit.'],permutations=[])
for order in itertools.permutations(range(9,13)):
    rows=[metrics(*state(r['raw_ticks'],order)) for r in records]
    Ra=np.array(rows[0]['tip_rotation_world']);Rb=np.array(rows[1]['tip_rotation_world'])
    report['permutations'].append(dict(order_root_to_tip=list(order),poses=[dict(pose=r['label'],**s) for r,s in zip(records,rows)],
        tip_translation_mm=float(np.linalg.norm(np.array(rows[1]['tip_world_m'])-rows[0]['tip_world_m'])*1000),
        tip_rotation_degrees=float(np.degrees(np.arccos(np.clip((np.trace(Ra.T@Rb)-1)/2,-1,1)))),
        within_limits_all_poses=all(not s['outside_limits'] for s in rows)))
selected=[('Existing 9,10,11,12',(9,10,11,12)),('Root-tip swapped 12,10,11,9',(12,10,11,9)),('Reversed 12,11,10,9',(12,11,10,9))]
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
canvas=Image.new('RGB',(2100,600*len(records)),'white');report['cases']=[]
for row,r in enumerate(records):
    for col,(label,order) in enumerate(selected):
        d,ids=state(r['raw_ticks'],order);case=dict(pose=r['label'],hypothesis=label,**metrics(d,ids));report['cases'].append(case)
        p=m.body('palm').id;R=d.xmat[p].reshape(3,3);o=d.xpos[p]
        view=R@np.array([.25,-1,-.25]);view/=np.linalg.norm(view)
        up=-R[:,2].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
        cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([0,-.02,-.04]);cam.distance=.4
        with mujoco.Renderer(m,height=600,width=700) as renderer:
            renderer.update_scene(d,camera=cam,scene_option=opt)
            for eye in renderer.scene.camera:eye.pos[:]=cam.lookat+view*.4;eye.forward[:]=-view;eye.up[:]=up
            im=Image.fromarray(renderer.render())
        draw=ImageDraw.Draw(im);draw.rectangle((0,0,700,80),fill='black')
        draw.text((10,8),r['label']+' | '+label,fill='white')
        draw.text((10,26),'Middle deg: '+', '.join(f'{x:.1f}' for x in case['middle_angles_deg']),fill='white')
        draw.text((10,44),f"Tip-thumb gap {case['tip_thumb']['gap_mm']:.1f}mm | limits: {case['outside_limits']}",fill='white')
        draw.text((10,62),'Offline candidate only; NO offsets, NO motor commands',fill='white')
        canvas.paste(im,(col*700,row*600))
canvas.save(OUT/'motor_order_comparison.jpg')
(OUT/'analysis.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(dict(permutations=len(report['permutations']),within_limits_all_poses=sum(x['within_limits_all_poses'] for x in report['permutations']),selected=[{k:c[k] for k in ('pose','hypothesis','middle_angles_deg','tip_thumb','outside_limits')} for c in report['cases']],out=str(OUT)),indent=2))
