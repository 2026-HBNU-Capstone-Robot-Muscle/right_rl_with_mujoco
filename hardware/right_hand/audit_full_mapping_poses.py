"""Offline paired-encoder reconstruction. Never imports a motor interface."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import hashlib
import itertools
import json
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image, ImageDraw
from mujoco_playground import registry
from box_clearance import clearance
from hand_motor_mapping import ticks_to_sim, metadata, LEGACY_SIM_TO_MOTOR_IDS, TICK
from hand_registration_geometry import candidate

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'full_mapping_pose_review_20261019'
OUT.mkdir(exist_ok=True)
SOURCES = [
    ('contact_0151', 'pose_pair_readonly_20261017_015118_321695'),
    ('side_0326', 'pose_pair_readonly_20261018_032605_137975'),
    ('tip_baseline', 'pose_pair_readonly_20261018_072023_698689/baseline'),
    ('tip_bent', 'pose_pair_readonly_20261018_072023_698689/bent'),
]
cfg = registry.get_default_config('LeapCubeFaceUp')
cfg.impl = 'jax'
m = registry.load('LeapCubeFaceUp', config=cfg).mj_model
palm = m.body('palm').id
hand = {palm}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:
        hand.add(b)
colors = {'if_': [.2,.55,1,1], 'mf_': [.1,.85,.2,1],
          'rf_': [.95,.7,.15,1], 'th_': [.95,.2,.35,1]}
boxes = []
for g in range(m.ngeom):
    body = m.body(int(m.geom_bodyid[g]))
    if body.id not in hand:
        m.geom_group[g] = 5
    for prefix, color in colors.items():
        if body.name.startswith(prefix):
            m.geom_rgba[g] = color
    if int(m.geom_type[g]) == 6 and (m.geom_contype[g] or m.geom_conaffinity[g]):
        boxes.append(g)
middle = [g for g in boxes if m.body(int(m.geom_bodyid[g])).name.startswith('mf_')]
thumb_housing = [g for g in boxes if m.body(int(m.geom_bodyid[g])).name in ('th_px','th_ds') and m.geom(g).name != 'th_tip']
ring = [g for g in boxes if m.body(int(m.geom_bodyid[g])).name.startswith('rf_')]
palmboxes = [g for g in boxes if int(m.geom_bodyid[g]) == palm]
registration = candidate()
Rwm = np.asarray(registration['hand_to_world_candidate'])
signs = np.asarray(list(itertools.product((-1,1),repeat=3)))

def nearest(d, left, right):
    rows = []
    for a in left:
        for b in right:
            gap, overlap = clearance(d.geom_xpos[a], d.geom_xmat[a].reshape(3,3), m.geom_size[a],
                                     d.geom_xpos[b], d.geom_xmat[b].reshape(3,3), m.geom_size[b])
            rows.append(dict(a=m.geom(a).name, b=m.geom(b).name,
                             gap_mm=float(gap*1000), overlap=bool(overlap)))
    return min(rows, key=lambda r:r['gap_mm']) if rows else None

def state(q):
    d = mujoco.MjData(m)
    for i in range(16):
        assert int(m.jnt_qposadr[int(m.actuator_trnid[i,0])]) == i
    d.qpos[:16] = q
    d.qpos[-7:-4] = [0,0,2]
    d.qpos[-4:] = [1,0,0,0]
    mujoco.mj_forward(m,d)
    return d

def metrics(d):
    hits = []
    for c in d.contact:
        a,b = int(c.geom1),int(c.geom2)
        if c.dist < -1e-5 and int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:
            hits.append(dict(a=m.geom(a).name,b=m.geom(b).name,overlap_mm=float(-1000*c.dist)))
    # Preserve the OLD anatomical base estimate only as a diagnostic transform.
    # Its key '9' was a legacy label for mf_mcp, not a measurement of physical ID9.
    marker_world = d.xanchor[int(m.actuator_trnid[4,0])] - Rwm @ np.asarray(registration['motor_axis_landmarks_in_marker_m']['9'])
    def marker(v):return ((np.asarray(v)-marker_world)@Rwm).tolist()
    return dict(joint_degrees=np.degrees(d.qpos[:16]).tolist(),
        boxes_marker_m=[dict(name=m.geom(g).name,body=m.body(int(m.geom_bodyid[g])).name,
            vertices=marker(d.geom_xpos[g]+(signs*m.geom_size[g])@d.geom_xmat[g].reshape(3,3).T))
            for g in boxes if int(m.geom_bodyid[g]) in hand],
        joints_marker_m={m.joint(int(m.actuator_trnid[i,0])).name:marker(d.xanchor[int(m.actuator_trnid[i,0])]) for i in range(16)},
        outside_limits=[m.joint(int(m.actuator_trnid[i,0])).name for i in range(16)
                        if not m.jnt_range[int(m.actuator_trnid[i,0]),0] <= d.qpos[i] <= m.jnt_range[int(m.actuator_trnid[i,0]),1]],
        middle_tip_thumb_housing=nearest(d,[m.geom('mf_tip').id],thumb_housing),
        whole_middle_thumb_housing=nearest(d,middle,thumb_housing),
        ring_palm=nearest(d,ring,palmboxes),native_model_overlaps=hits)

opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
def render(d, view_vector):
    R=d.xmat[palm].reshape(3,3);o=d.xpos[palm]
    view=R@np.array(view_vector,dtype=float);view/=np.linalg.norm(view)
    up=-R[:,2].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
    cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([0,-.02,-.06]);cam.distance=.49
    with mujoco.Renderer(m,height=480,width=640) as ren:
        ren.update_scene(d,camera=cam,scene_option=opt)
        for eye in ren.scene.camera:
            eye.pos[:]=cam.lookat+view*.49;eye.forward[:]=-view;eye.up[:]=up
        return Image.fromarray(ren.render())

report=dict(mapping=metadata(),hardware_commands_sent=0,calibration_verified=False,
    marker_transform_status='Historical anatomy-based estimate only; legacy ID labels are not physical ID landmarks.',
    limitations=['Static forward kinematics, not dynamics or SAC rollout.',
                 'User-supplied physical order; common raw angle minus pi convention has not been calibrated.',
                 'Views are illustrative, not registered pixel overlays. No fitted offsets or sign changes.',
                 'Collision primitives are approximate; overlaps/gaps do not prove physical contact.',
                 'Before/after agreement verifies endpoints, not continuous stationarity.'],poses=[])
for label,source in SOURCES:
    folder=ROOT/source
    r=json.loads((folder/'result.json').read_text(encoding='utf-8'))
    snapshots=r['snapshots'];assert len(snapshots)==2
    raw=[]
    for snap in snapshots:
        motors=sorted(snap['motors'],key=lambda v:v['id'])
        assert [v['id'] for v in motors]==list(range(1,17))
        raw.append([v['position_ticks_signed'] for v in motors])
    drift=np.max(np.abs(np.array(raw[0])-raw[1]));assert drift<=3
    raw=np.mean(raw,axis=0)
    old=raw[np.array(LEGACY_SIM_TO_MOTOR_IDS)-1]*TICK-np.pi
    new=ticks_to_sim(raw)
    sheet=Image.new('RGB',(1920,960),'white')
    photo=Image.open(folder/'paired_photo.png').convert('RGB')
    photo.thumbnail((640,870));sheet.paste(photo,((640-photo.width)//2,60))
    draw=ImageDraw.Draw(sheet);draw.text((15,10),label+' | saved paired photo',fill='black')
    row=dict(label=label,source=source,source_sha256=hashlib.sha256((folder/'result.json').read_bytes()).hexdigest(),
             photo_sha256=hashlib.sha256((folder/'paired_photo.png').read_bytes()).hexdigest(),
             endpoint_drift_ticks=float(drift),raw_ticks=raw.tolist(),interpretations={})
    for col,(name,q) in enumerate([('Legacy mapping',old),('User confirmed mapping',new)],start=1):
        d=state(q);met=metrics(d);row['interpretations'][name]=met
        for viewnum,view in enumerate(([.25,-1,-.25],[1,-.25,-.4])):
            im=render(d,view);di=ImageDraw.Draw(im);di.rectangle((0,0,640,70),fill='black')
            di.text((8,6),name+' | view '+str(viewnum+1),fill='white')
            gap=met['whole_middle_thumb_housing']['gap_mm']
            di.text((8,24),f'Middle / thumb housing gap: {gap:.2f} mm (model only)',fill='white')
            di.text((8,42),'Blue index / green middle / gold ring / red thumb',fill='white')
            sheet.paste(im,(col*640,viewnum*480))
    sheet.save(OUT/(label+'_comparison.jpg'))
    report['poses'].append(row)
(OUT/'analysis.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({p['label']:{k:{key:v[key] for key in ('whole_middle_thumb_housing','ring_palm','outside_limits')}
                            for k,v in p['interpretations'].items()} for p in report['poses']},indent=2))
