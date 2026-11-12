"""Offline saved-pose comparison after the user moved the rigid support edge.

No trajectory is executed or approved. Keep the old 34 mm geometry only as an
explicit comparison; the default builder must now use the reported 50 mm.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont
from mujoco_playground import registry
from sac_wood_glass_model import build
from hand_motor_mapping import ticks_to_sim, SIM_TO_MOTOR_IDS
from grasp_place_plan import endpoints

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024'/'support_50mm_review'
SOURCE=ROOT/'pose_pair_readonly_20261024_045449_430810'/'result.json'
OUT.mkdir(parents=True,exist_ok=True)
blob=SOURCE.read_bytes();r=json.loads(blob)
assert r['status']=='photo_and_readings_saved'
assert r['pair_check']['maximum_drift_ticks']<=3
assert r['servo_write_commands_sent']==0 and r['torque_changes_requested']==0
raw=np.array([next(v['position_ticks_signed'] for v in r['snapshots'][0]['motors'] if v['id']==i) for i in range(1,17)])
opened,closed=endpoints()
measurement=json.loads((ROOT/'wood_glass_support_measurements.json').read_text(encoding='utf-8'))
assert measurement['wood_projection_from_fixed_motor_mount_front_mm']==50.
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg)

def contacts(model,data):
    support={model.geom(n).id for n in ('support_glass','support_wood')}
    palm=model.body('palm').id;hand={palm}
    for b in range(model.nbody):
        if int(model.body_parentid[b]) in hand:hand.add(b)
    pairs={}
    for c in data.contact:
        a,b=int(c.geom1),int(c.geom2)
        if c.dist>=0:continue
        if not ((a in support and int(model.geom_bodyid[b]) in hand)
                or (b in support and int(model.geom_bodyid[a]) in hand)):continue
        key='|'.join(sorted((model.geom(a).name,model.geom(b).name)))
        pairs[key]=max(pairs.get(key,0.),-float(c.dist)*1000)
    return pairs

def put(model,data,q):
    data.qpos[:16]=ticks_to_sim(q)
    data.qpos[-7:-4]=[0,0,2];data.qpos[-4:]=[1,0,0,0]
    mujoco.mj_forward(model,data)

records=[];rendered={}
fontpath=Path('/mnt/c/Windows/Fonts/malgun.ttf')
boldpath=Path('/mnt/c/Windows/Fonts/malgunbd.ttf')
font=ImageFont.truetype(str(fontpath),21)
bold=ImageFont.truetype(str(boldpath),27)
for projection in (34.,50.):
    # None deliberately exercises the production builder's new default.
    m,manifest=build(env,projection_mm=34. if projection==34. else None)
    assert manifest['projection_mm']==projection
    assert m.nq==env.mj_model.nq and m.nu==env.mj_model.nu
    assert np.array_equal(m.jnt_range,env.mj_model.jnt_range)
    assert np.array_equal(m.actuator_ctrlrange,env.mj_model.actuator_ctrlrange)
    d=mujoco.MjData(m);poses=[]
    for label,q in (('paired_off',raw),('historical_open_endpoint',opened),('historical_grasp_endpoint',closed)):
        put(m,d,q);pairs=contacts(m,d)
        poses.append(dict(name=label,raw_ticks=q.tolist(),support_contacts_mm=pairs,
                          maximum_model_overlap_mm=max(pairs.values(),default=0.)))
    # One-joint +/-4 observations bound sensitivity only at this saved pose.
    offsets=[np.zeros(16,dtype=int)]
    for i in range(16):
        for sign in (-1,1):
            off=np.zeros(16,dtype=int);off[i]=4*sign;offsets.append(off)
    nearby=[]
    for off in offsets:
        put(m,d,raw+off);pairs=contacts(m,d)
        nearby.append(dict(offset_ticks=off.tolist(),maximum_model_overlap_mm=max(pairs.values(),default=0.)))
    records.append(dict(projection_mm=projection,manifest=manifest,poses=poses,
                        saved_pose_neighborhood=nearby))
    put(m,d,raw)
    palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(name).id for name in ('support_glass','support_wood')}
    for g in range(m.ngeom):
        name=m.geom(g).name or ''
        # Cube visual sub-geometries have unnamed/material-driven parts. Hide
        # every unrelated visual by group instead of relying on alpha/name.
        if int(m.geom_bodyid[g]) not in hand and g not in support:m.geom_group[g]=5
        body=m.body(int(m.geom_bodyid[g])).name or ''
        for prefix,color in {'if_':[.15,.48,.95,1],'mf_':[.15,.72,.3,1],
                             'rf_':[.95,.68,.1,1],'th_':[.94,.2,.32,1]}.items():
            if body.startswith(prefix):m.geom_rgba[g]=color
    palm=m.body('palm').id;rotation=d.xmat[palm].reshape(3,3);origin=d.xpos[palm]
    opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3]=0;opt.geomgroup[4]=0;opt.geomgroup[5]=0
    m.vis.headlight.ambient[:]=[.5,.5,.5]
    m.vis.headlight.diffuse[:]=[.7,.7,.7]
    views=[]
    for title,eye_local in (('옆에서 본 모델',[.05,-1.,-.08]),('비스듬히 본 모델',[.6,-1.,-.8])):
        view=rotation@np.array(eye_local);view/=np.linalg.norm(view)
        up=-rotation[:,2];up=up-view*np.dot(up,view);up/=np.linalg.norm(up)
        camera=mujoco.MjvCamera();camera.lookat[:]=origin+rotation@np.array([.005,-.025,-.005]);camera.distance=.43
        with mujoco.Renderer(m,height=490,width=850) as renderer:
            renderer.update_scene(d,camera=camera,scene_option=opt)
            for eye in renderer.scene.camera:
                eye.pos[:]=camera.lookat+view*.43;eye.forward[:]=-view;eye.up[:]=up
            im=Image.fromarray(renderer.render())
        panel=Image.new('RGB',(850,550),'#f5f7fb');panel.paste(im,(0,60))
        draw=ImageDraw.Draw(panel)
        draw.text((20,6),f'{"이전 배치" if projection==34 else "현재 배치"}: 약 {projection:.0f}mm / {title}',font=bold,fill='#142338')
        draw.text((20,36),'저장된 같은 관절각으로 비교. 실물 접촉 측정이나 실행 승인이 아님.',font=font,fill='#53647a')
        views.append(panel)
    rendered[int(projection)]=views

canvas=Image.new('RGB',(1700,1190),'#f5f7fb')
for col,projection in enumerate((34,50)):
    for row,im in enumerate(rendered[projection]):canvas.paste(im,(col*850,row*550))
draw=ImageDraw.Draw(canvas)
draw.text((25,1109),'파랑=검지  초록=중지  노랑=약지  빨강=엄지 / 청록=유리  갈색=나무',font=bold,fill='#142338')
draw.text((25,1153),'받침의 폭과 뒤쪽 길이 등은 아직 가정값이다. 모델 겹침의 mm를 실제 관통 깊이로 읽으면 안 된다.',font=font,fill='#53647a')
canvas.save(OUT/'support_34_vs_50_same_pose.jpg',quality=92)
report=dict(status='offline_support_50mm_comparison_complete',
    source=str(SOURCE.relative_to(ROOT)),source_sha256=hashlib.sha256(blob).hexdigest(),
    measurement_source='User current-setup clarification, 2026-10-24',
    measurement_sha256=hashlib.sha256((ROOT/'wood_glass_support_measurements.json').read_bytes()).hexdigest(),
    mapping=list(SIM_TO_MOTOR_IDS),comparisons=records,
    scope='Static saved-pose and endpoint diagnostic only, no dynamics or full path certificate.',
    native_ranges_unchanged=True,hardware_commands_sent=0,real_devices_opened=False,
    hardware_execution_allowed=False,physical_calibration_verified=False,
    previous_34mm_certificates_apply_to_current_setup=False,
    pending_user_observation='Side photo showing the fixed body underside and the rigid glass/wood front edge together; no motor execution or precise remeasurement requested.',
    limitations=['Current projection is approximate. Glass thickness and front alignment are retained from earlier reports.',
                'Support width, rear reach and wood depth remain unmeasured conservative extents.',
                'No physical collision depth is inferred from model overlaps.'])
(OUT/'comparison.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(dict(status=report['status'],cases=[dict(projection_mm=c['projection_mm'],
    pose_maxima_mm={p['name']:round(p['maximum_model_overlap_mm'],4) for p in c['poses']},
    neighborhood_maximum_mm=max(s['maximum_model_overlap_mm'] for s in c['saved_pose_neighborhood'])) for c in records],
    output=str(OUT),hardware_execution_allowed=False),indent=2),flush=True)
