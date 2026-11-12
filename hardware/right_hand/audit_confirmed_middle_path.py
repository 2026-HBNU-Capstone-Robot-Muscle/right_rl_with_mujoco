"""Offline interpretation/remapping of historical commands; no device access."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json,argparse
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from mujoco_playground import registry
from hand_motor_mapping import ticks_to_sim,sim_to_ticks,metadata,LEGACY_SIM_TO_MOTOR_IDS,TICK
from sac_wood_glass_model import build

ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--out',default='full_mapping_review_20261018')
args=parser.parse_args();OUT=ROOT/args.out;OUT.mkdir(exist_ok=True)
pathfile=ROOT/'sac_reachable_empty_path_id16capped.json'
old=json.loads(pathfile.read_text(encoding='utf-8'))
ticks=np.array([old['start_ticks']]+old['goals_id_order'])
intended=ticks[:,np.array(LEGACY_SIM_TO_MOTOR_IDS)-1]*TICK-np.pi
as_sent=ticks_to_sim(ticks);remapped=sim_to_ticks(intended)
assert np.max(np.abs(ticks_to_sim(remapped)-intended))<1e-12
assert sorted(metadata()['motor_ids_in_sim_order'])==list(range(1,17))
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env)
palm=m.body('palm').id;hand={palm}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={m.geom(n).id for n in ('support_glass','support_wood')}
for g in range(m.ngeom):
    if int(m.geom_bodyid[g]) not in hand and g not in support:m.geom_group[g]=5
    for prefix,col in {'if_':[.2,.55,1,1],'mf_':[.1,.85,.2,1],'rf_':[.95,.7,.15,1],'th_':[.95,.2,.35,1]}.items():
        if m.body(int(m.geom_bodyid[g])).name.startswith(prefix):m.geom_rgba[g]=col

def evaluate(q):
    d=mujoco.MjData(m);d.qpos[:16]=q;d.ctrl[:]=q
    d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
    hits=[]
    for c in d.contact:
        if c.dist>=-1e-5:continue
        a,b=int(c.geom1),int(c.geom2)
        ha=int(m.geom_bodyid[a]) in hand;hb=int(m.geom_bodyid[b]) in hand
        if not ((ha and hb) or (ha and b in support) or (hb and a in support)):continue
        hits.append(dict(a=m.geom(a).name,b=m.geom(b).name,penetration_mm=float(-c.dist*1000),
                         support=a in support or b in support))
    limits=[m.joint(int(m.actuator_trnid[i,0])).name for i in range(16) if not m.jnt_range[int(m.actuator_trnid[i,0]),0]-1e-6<=q[i]<=m.jnt_range[int(m.actuator_trnid[i,0]),1]+1e-6]
    return d,dict(contacts=hits,outside_limits=limits)

def sweep(qs,labels=None):
    rows=[]
    for i,q in enumerate(qs):
        _,r=evaluate(q);rows.append(dict(index=i,phase=labels[i] if labels else None,**r))
    return dict(samples=len(rows),self_contact_samples=sum(any(not c['support'] for c in r['contacts']) for r in rows),
                support_contact_samples=sum(any(c['support'] for c in r['contacts']) for r in rows),
                outside_limits_samples=sum(bool(r['outside_limits']) for r in rows),
                max_model_overlap_mm=max([c['penetration_mm'] for r in rows for c in r['contacts']]or[0]),rows=rows)

recordfile=ROOT/'leap_id12_backoff_empty_result.json';record=json.loads(recordfile.read_text(encoding='utf-8'))
measured=ticks_to_sim(np.array([r['actual'] for r in record['samples']]))
report=dict(status='offline_mapping_corrected_path_audit',mapping=metadata(),hardware_commands_sent=0,
    motor_execution_ready=False,source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (pathfile,recordfile,ROOT/'hand_motor_mapping.py')},
    obstacle_model=manifest,old_start_ticks=ticks[0].tolist(),remapped_start_ticks_offline_only=remapped[0].tolist(),
    joint_mismatch_max_degrees=np.max(np.abs(as_sent-intended),axis=0).__mul__(180/np.pi).tolist(),
    root_motor_ids=[metadata()['motor_ids_in_sim_order'][i] for i in (0,4,8)],old_root_motor_ids=[5,9,13],
    historical_commands_interpreted_with_confirmed_mapping=sweep(as_sent),
    original_intent_with_corrected_encoding=sweep(ticks_to_sim(remapped)),
    measured_backoff_interpreted_with_confirmed_mapping=sweep(measured,[r['phase'] for r in record['samples']]),
    limitations=['Static kinematic collision sweep, not dynamics replay or SAC training.',
                 'All four finger orders supplied by user; signs/zeros and housing registration not fully calibrated.',
                 'Native engine collision filters/primitives apply; positive model overlap is not measured physical penetration.',
                 'Preparation trajectories and role-specific gains must be revalidated; old audits cannot approve new mapping.',
                 'Explicit per-finger user order used, not an inferred uniform reversal.'])
(OUT/'path_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
(OUT/'remapped_path_OFFLINE_ONLY.json').write_text(json.dumps(dict(format='offline_mapping_review_not_executable_v1',
    hardware_execution_allowed=False,mapping=metadata(),source_sha256=report['source_sha256'][pathfile.name],
    start_ticks=remapped[0].tolist(),goals_id_order=remapped[1:].tolist(),
    warning='Not accepted by any hardware loader; preparation and dynamics pending.'),indent=2),encoding='utf-8')

opt=mujoco.MjvOption();opt.geomgroup[2]=1;opt.geomgroup[3:]=0
sample_indices=[0,20,40,len(ticks)-1]
sheet=Image.new('RGB',(1400,600*len(sample_indices)),'white')
for row,k in enumerate(sample_indices):
    for col,(label,qs) in enumerate([('Historical commands: actual joint interpretation',as_sent),('Original model intent: corrected ID conversion',intended)]):
        d,r=evaluate(qs[k]);R=d.xmat[palm].reshape(3,3);o=d.xpos[palm]
        view=R@np.array([.25,-1,-.25]);view/=np.linalg.norm(view)
        up=-R[:,2].copy();up-=view*np.dot(up,view);up/=np.linalg.norm(up)
        cam=mujoco.MjvCamera();cam.lookat[:]=o+R@np.array([0,-.02,-.04]);cam.distance=.4
        with mujoco.Renderer(m,height=600,width=700) as ren:
            ren.update_scene(d,camera=cam,scene_option=opt)
            for eye in ren.scene.camera:eye.pos[:]=cam.lookat+view*.4;eye.forward[:]=-view;eye.up[:]=up
            im=Image.fromarray(ren.render())
        draw=ImageDraw.Draw(im);draw.rectangle((0,0,700,75),fill='black')
        draw.text((10,8),f'Sample {k}/{len(ticks)-1} | '+label,fill='white')
        draw.text((10,27),'Middle deg: '+', '.join(f'{x:.1f}' for x in np.degrees(qs[k,4:8])),fill='white')
        draw.text((10,47),f"OFFLINE ONLY | model overlap contacts: {len(r['contacts'])} | no cube / motors",fill='white')
        sheet.paste(im,(col*700,row*600))
sheet.save(OUT/'historical_vs_intended_path.jpg')
print(json.dumps({k:({a:b for a,b in v.items() if a!='rows'} if isinstance(v,dict) else v) for k,v in report.items() if k in ('joint_mismatch_max_degrees','old_start_ticks','remapped_start_ticks_offline_only','historical_commands_interpreted_with_confirmed_mapping','original_intent_with_corrected_encoding','measured_backoff_interpreted_with_confirmed_mapping')},indent=2))
