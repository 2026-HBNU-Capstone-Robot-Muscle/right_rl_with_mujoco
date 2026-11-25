"""Offline current-pose and preparation screening; no device imports or writes."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from mujoco_playground import registry
from hand_motor_mapping import ticks_to_sim,metadata,TICK
from offline_housing_proxy_model import build_with_middle_thumb_hulls

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_start_20261019';OUT.mkdir(exist_ok=True)
source=ROOT/'hand_readonly_check_result.json';content=source.read_bytes();r=json.loads(content)
assert r['status']=='read_complete_not_motion_approval'
motors=sorted(r['motors'],key=lambda x:x['id']);assert [x['id'] for x in motors]==list(range(1,17))
assert all(x['torque_enabled']==x['torque_enabled_end']==x['hardware_error']==0 for x in motors)
raw=np.array([x['position_ticks_signed'] for x in motors]);q=ticks_to_sim(raw)
assert np.allclose(q,r['sim_angles_rad'])
(OUT/'source_readonly.json').write_bytes(content)
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build_with_middle_thumb_hulls(env)
joint_ids=m.actuator_trnid[:,0];bounds=m.jnt_range[joint_ids].copy()
pathfile=ROOT/'full_mapping_pose_review_20261019/inward_quantized/housing_sensitivity/path_OFFLINE_ONLY.json'
candidate=json.loads(pathfile.read_text());target=ticks_to_sim(candidate['start_ticks'])
palm=m.body('palm').id;hand={palm}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
supports={m.geom(n).id for n in ('support_glass','support_wood')}
def pose(angles):
    d=mujoco.MjData(m);d.qpos[:16]=angles;d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
    mujoco.mj_forward(m,d);contacts=[]
    for c in d.contact:
        a,b=int(c.geom1),int(c.geom2)
        if c.dist>=-1e-5:continue
        if ((int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand)
            or (a in supports and int(m.geom_bodyid[b]) in hand)
            or (b in supports and int(m.geom_bodyid[a]) in hand)):
            contacts.append(dict(a=m.geom(a).name,b=m.geom(b).name,overlap_mm=float(-c.dist*1000),support=a in supports or b in supports))
    violations=[]
    for i in range(16):
        if not bounds[i,0]<=angles[i]<=bounds[i,1]:
            violations.append(dict(joint=m.joint(int(joint_ids[i])).name,motor_id=metadata()['motor_ids_in_sim_order'][i],
                angle=float(angles[i]),lower=float(bounds[i,0]),upper=float(bounds[i,1]),
                excess_degrees=float(np.degrees(max(bounds[i,0]-angles[i],angles[i]-bounds[i,1])))))
    return d,dict(contacts=contacts,model_range_violations=violations)

def screen(waypoints):
    last=q.copy();rows=[]
    for name,end in waypoints:
        n=max(1,int(np.ceil(np.max(abs(end-last))/.015)))
        for k in range(1,n+1):
            a=last+(end-last)*k/n;_,metrics=pose(a)
            rows.append(dict(segment=name,fraction=k/n,**metrics))
        last=end.copy()
    return dict(samples=len(rows),model_overlap_samples=sum(bool(x['contacts']) for x in rows),
        support_overlap_samples=sum(any(c['support'] for c in x['contacts']) for x in rows),
        max_support_overlap_mm=max([c['overlap_mm'] for x in rows for c in x['contacts'] if c['support']]or[0]),
        max_self_overlap_mm=max([c['overlap_mm'] for x in rows for c in x['contacts'] if not c['support']]or[0]),
        outside_model_range_samples=sum(bool(x['model_range_violations']) for x in rows),rows=rows,
        hardware_execution_allowed=False)
thumb=q.copy();thumb[12:]=target[12:]
root=q.copy();root[[0,4,8]]=np.maximum(root[[0,4,8]],.65)
rootthumb=root.copy();rootthumb[12:]=target[12:]
_,current=pose(q)
report=dict(status='offline_preparation_screen_not_executable',source_timestamp=r['timestamp'],
    source_sha256=hashlib.sha256(content).hexdigest(),mapping=metadata(),raw_ticks=raw.tolist(),sim_angles_rad=q.tolist(),
    current=current,model_manifest=manifest,motor_commands_sent=0,hardware_execution_allowed=False,
    target_start_ticks=candidate['start_ticks'],target_source_sha256=hashlib.sha256(pathfile.read_bytes()).hexdigest(),
    physical_joint_zero_calibrated=False,wrap_analysis=[],screens={},
    limitations=['Kinematic target sweep only, not a motor trajectory or dynamics approval.',
                 'No clipping, wrapping, fitted offsets or expanded limits applied to the measured start.',
                 'Static overlaps in approximate geometry are not physical penetration measurements.',
                 'Observed mode 5 is multi-turn; modulo conversion is not a justified hardware correction.'])
for i in range(16):
    if q[i]<bounds[i,0] or q[i]>bounds[i,1]:
        report['wrap_analysis'].append(dict(motor_id=metadata()['motor_ids_in_sim_order'][i],joint=m.joint(int(joint_ids[i])).name,
            equivalents=[dict(turns=k,angle=float(q[i]+2*np.pi*k),within_model_range=bool(bounds[i,0]<=q[i]+2*np.pi*k<=bounds[i,1])) for k in (-1,0,1)]))
for name,waypoints in [('direct',[('direct',target)]),('thumb_first',[('thumb',thumb),('others',target)]),
                       ('roots_thumb_others',[('roots',root),('thumb',rootthumb),('others',target)])]:
    report['screens'][name]=screen(waypoints)
(OUT/'analysis.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
colors={'if_':[.2,.55,1,1],'mf_':[.1,.85,.2,1],'rf_':[.95,.7,.15,1],'th_':[.95,.2,.35,1]}
for g in range(m.ngeom):
    body=m.body(int(m.geom_bodyid[g]))
    if body.id not in hand and g not in supports:m.geom_group[g]=5
    for prefix,color in colors.items():
        if body.name.startswith(prefix):m.geom_rgba[g]=color
opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
sheet=Image.new('RGB',(1800,1200),'white')
for row,(title,angles) in enumerate([('CURRENT measured encoders; ID3 outside model range',q),('SIMULATION target start; NOT a hardware command',target)]):
    d,_=pose(angles);R=d.xmat[palm].reshape(3,3);o=d.xpos[palm]
    for col,vector in enumerate(([.3,-1,-.4],[1,.3,-.5],[.2,-.2,-1])):
        view=R@np.array(vector,dtype=float);view/=np.linalg.norm(view)
        up=-R[:,2].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
        cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([0,-.02,-.06]);cam.distance=.46
        with mujoco.Renderer(m,height=600,width=600) as ren:
            ren.update_scene(d,camera=cam,scene_option=opt)
            for eye in ren.scene.camera:eye.pos[:]=cam.lookat+view*.46;eye.forward[:]=-view;eye.up[:]=up
            im=Image.fromarray(ren.render())
        draw=ImageDraw.Draw(im);draw.rectangle((0,0,600,65),fill='black')
        draw.text((8,8),title,fill='white');draw.text((8,26),'Blue index / green middle / gold ring / RED THUMB',fill='white')
        draw.text((8,44),'Offline model only; signs/zero and actual contact unverified',fill='white')
        sheet.paste(im,(col*600,row*600))
sheet.save(OUT/'current_and_target_views.jpg')
print(json.dumps(dict(timestamp=r['timestamp'],raw_ticks=raw.tolist(),current=current,wrap_analysis=report['wrap_analysis'],
    screens={name:{k:v for k,v in s.items() if k!='rows'} for name,s in report['screens'].items()}),indent=2))
