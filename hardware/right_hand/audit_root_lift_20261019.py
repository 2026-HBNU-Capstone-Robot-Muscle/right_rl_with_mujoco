"""Offline exact-integer root-lift screening. No serial/runner imports."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib
import argparse
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from hand_motor_mapping import ticks_to_sim
from offline_housing_proxy_model import build_with_middle_thumb_hulls

root=Path(__file__).resolve().parent
parser=argparse.ArgumentParser()
parser.add_argument('--preflight-1811',action='store_true')
parser.add_argument('--thumb-shift',action='store_true')
args=parser.parse_args()
is_preflight=args.preflight_1811 or args.thumb_shift
out=root/('right_hand_start_20261019_thumb_shift' if args.thumb_shift else
          'right_hand_start_20261019_1811' if args.preflight_1811 else 'right_hand_start_20261019')
source=(out/('source_preflight.json' if is_preflight else 'source_readonly.json')).read_bytes()
record=json.loads(source)
if is_preflight:
    if args.preflight_1811:assert record['timestamp']=='2026-10-19T18:11:01.488208+09:00'
    assert record['write_events']==[] and record['all_torque_off_verified']
    state=record['preflight_state']
    assert not any(state['torque']) and not any(state['error']) and not any(state['watchdog'])
    raw=np.array(state['position'])
else:
    assert hashlib.sha256(source).hexdigest()=='faa0c6227b8f14dc68a3cd8d928837f6f72709c7bb303b8a34e67b53917b74d8'
    raw=np.array([m['position_ticks_signed'] for m in record['motors']])
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg)
m,manifest=build_with_middle_thumb_hulls(env)
palm=m.body('palm').id;hand={palm}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
supports={m.geom(n).id for n in ('support_glass','support_wood')}
d=mujoco.MjData(m)
d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]

def sample(a):
    d.qpos[:16]=ticks_to_sim(a);mujoco.mj_forward(m,d)
    support=[];self_depth=[]
    for c in d.contact:
        a,b=int(c.geom1),int(c.geom2)
        if c.dist>=-1e-5:continue
        ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
        if ha and hb:self_depth.append(float(-c.dist*1000))
        if (ha and b in supports) or (hb and a in supports):support.append(float(-c.dist*1000))
    return max(support or [0]),max(self_depth or [0])

rng=np.random.default_rng(19)
starts=[raw]+[raw+rng.integers(-16,17,16) for _ in range(32)]
rows=[]
for trial,start in enumerate(starts):
    depths=[]
    for step in range(121):
        a=start.copy();idx=np.array([7,11,15])
        a[idx]=np.rint(start[idx]+(2472-start[idx])*step/120).astype(int)
        depths.append(sample(a))
    depths=np.array(depths)
    rows.append(dict(trial=trial,initial_ticks=start.tolist(),
        max_self_overlap_mm=float(depths[:,1].max()),
        initial_support_overlap_mm=float(depths[0,0]),
        final_support_overlap_mm=float(depths[-1,0]),
        largest_support_depth_increase_mm=float(max(0,np.diff(depths[:,0]).max())),
        support_depth_nonincreasing=bool(np.all(np.diff(depths[:,0])<=.01))))
result=dict(status='kinematic_screen_only_not_physical_clearance_certification',
    exact_nominal_steps=120,source_sha256=hashlib.sha256(source).hexdigest(),
    model_manifest=manifest,trial_count=len(rows),rows=rows,
    worst_self_overlap_mm=max(x['max_self_overlap_mm'] for x in rows),
    worst_final_support_overlap_mm=max(x['final_support_overlap_mm'] for x in rows),
    all_support_depth_nonincreasing=all(x['support_depth_nonincreasing'] for x in rows),
    motor_commands_sent=0,thumb_range_verified=False,
    limitations=['Static placements, no dynamics or servo-load validation.',
                 'Support placement approximate; initial model overlap is not measured physical penetration.',
                 '32 random perturbations are sensitivity checks, not exhaustive bounds.',
                 'ID3 remains outside native model range and is not swept or wrapped.'])
(out/'integer_root_lift_screen.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('rows','model_manifest')},indent=2))
