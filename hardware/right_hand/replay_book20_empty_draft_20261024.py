"""Offline dynamic sensitivity of the selected EMPTY preparation draft.

Book front remains an explicit hypothesis. No resulting file is a motor proof.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import argparse
import hashlib
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from hand_motor_mapping import ticks_to_sim

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/book20_current_review'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--front-mm',nargs='+',type=float)
parser.add_argument('--output-name',default='dynamic_diagnostic.json')
args=parser.parse_args()
if Path(args.output_name).name!=args.output_name or not args.output_name.endswith('.json'):
    raise ValueError('Diagnostic output must be a JSON basename')
fronts=args.front_mm or (-20.,0.)
if not np.isfinite(fronts).all():raise ValueError('Invalid diagnostic fronts')
c=json.loads((OUT/'timing_selected_offline_goals.json').read_bytes())
assert c['hardware_execution_allowed'] is False
raw=np.array(c['raw_ticks']);rows=[(p,np.array(q)) for p,q in c['goals']]
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
records=[]
for front in fronts:
    m,manifest=build_book(env,OUT/'dynamic_scene.xml',book_projection_mm=front,
        book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    gain=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy()
    for scale in (.5,1.,1.5):
        m.actuator_gainprm[:]=gain;m.actuator_biasprm[:]=bias
        m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
        d.ctrl[:]=ticks_to_sim(raw);mujoco.mj_forward(m,d)
        maximum_tracking=maximum_support=maximum_self=after_roots=0.;phases={};finite=True
        path=[('initial_hold',raw)]*10+rows+[('final_hold',rows[-1][1])]*20
        for phase,q in path:
            goal=ticks_to_sim(q);d.ctrl[:]=goal
            for _ in range(round(.05/m.opt.timestep)):
                mujoco.mj_step(m,d)
                if not np.isfinite(d.qpos).all():finite=False;break
                tracking=float(max(abs(d.qpos[:16]-goal)));sp=sm=0.
                for contact in d.contact:
                    if contact.dist>=0:continue
                    a,b=int(contact.geom1),int(contact.geom2)
                    ha=int(m.geom_bodyid[a]) in hand;hb=int(m.geom_bodyid[b]) in hand
                    if (a in support and hb) or (b in support and ha):sp=max(sp,-float(contact.dist)*1000)
                    if ha and hb:sm=max(sm,-float(contact.dist)*1000)
                maximum_tracking=max(maximum_tracking,tracking);maximum_support=max(maximum_support,sp);maximum_self=max(maximum_self,sm)
                if phase in ('thumb_and_fingers','final_hold'):after_roots=max(after_roots,sp)
                row=phases.setdefault(phase,dict(max_tracking_rad=0.,max_support_model_overlap_mm=0.,max_self_model_overlap_mm=0.))
                row['max_tracking_rad']=max(row['max_tracking_rad'],tracking)
                row['max_support_model_overlap_mm']=max(row['max_support_model_overlap_mm'],sp)
                row['max_self_model_overlap_mm']=max(row['max_self_model_overlap_mm'],sm)
            if not finite:break
        record=dict(book_front_mm_hypothesis=front,gain_scale=scale,finite=finite,
            max_tracking_rad=maximum_tracking,max_support_model_overlap_mm=maximum_support,
            max_self_model_overlap_mm=maximum_self,max_support_after_root_phase_mm=after_roots,
            phases=phases,hardware_execution_allowed=False)
        records.append(record)
        print(json.dumps({k:v for k,v in record.items() if k!='phases'}),flush=True)
report=dict(status='offline_dynamic_hypotheses_complete',source_sha256=c['source_sha256'],
    nominal_path_seconds=len(rows)*.05,book_thickness_mm_user_reported=20.,records=records,
    current_measurement_sha256=hashlib.sha256((ROOT/'current_book_support_measurements_20261024.json').read_bytes()).hexdigest(),
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    native_ranges_unchanged=True,motor_commands_sent=0,real_devices_opened=False,
    hardware_execution_allowed=False,physical_calibration_verified=False,
    limitations=['Book front hypotheses have not been confirmed by the user.',
        'Initial support model overlap is present and unapproved; this is not an accepted external-contact escape certificate.',
        'Controller scales are simulated responses, not identified motor dynamics or changed real PID.',
        'No start neighborhood certificate or real motion approval is issued.'])
(OUT/args.output_name).write_text(json.dumps(report,indent=2),encoding='utf-8')
print('Saved offline dynamic diagnostic; hardware execution remains disallowed.',flush=True)
