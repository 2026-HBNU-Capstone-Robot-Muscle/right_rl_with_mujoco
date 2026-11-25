"""Check that a candidate flip is supported by hand rather than the mount."""
import argparse,json
from pathlib import Path
import mujoco
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--mjx',action='store_true');p.add_argument('--run');args=p.parse_args()
name='sac_supported_motion_search_20hz_result'
if args.run:args.mjx=True
source=ROOT/args.run/'supported_batch_eval.json' if args.run else ROOT/(name+('_mjx_eval' if args.mjx else '')+'.json')
r=json.loads(source.read_text())
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';m,_=build(registry.load('LeapCubeFaceUp',config=cfg));d=mujoco.MjData(m);cube=m.geom('cube').id
rows=r['results'] if args.mjx else [dict(offset_index=0,seed=0,trace=r['validation'][0]['trace'])]
results=[]
for row in rows:
    samples=[];longest=streak=0
    for rec in row['trace']:
        if not rec['step']:continue
        d.qpos[:]=rec['qpos'];mujoco.mj_forward(m,d)
        partners=sorted({m.geom(c.geom2 if c.geom1==cube else c.geom1).name for c in d.contact if cube in (c.geom1,c.geom2) and c.dist<=.0001})
        error=rec['error'] if args.mjx else rec['target_error_deg'];shift=rec['shift'] if args.mjx else rec['shift_mm']
        hand=any(n.startswith(('palm_','if_','mf_','rf_','th_')) for n in partners)
        external=any(n.startswith(('support_','floor','leap_mount')) for n in partners)
        streak=streak+1 if error<=25 and shift<=30 and hand and not external else 0;longest=max(longest,streak)
        if error<=25 or rec is row['trace'][-1]:samples.append(dict(step=rec['step'],target_error_deg=error,shift_mm=shift,partners=partners))
    results.append(dict(offset_index=row['offset_index'],seed=row['seed'],longest_hand_only_hold_samples=longest,samples=samples))
report=dict(status='native_geometry_contact_reconstruction',source=str(source),mjx_trace=args.mjx,results=results,motor_commands_sent=0,
            caveat='A reconstruction audit, not measured contact or equivalent to MJX dynamics.')
output=ROOT/args.run/'contact_audit.json' if args.run else ROOT/('sac_reference_contact_audit'+('_mjx' if args.mjx else '')+'_result.json')
output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps([dict(offset=r['offset_index'],seed=r['seed'],longest=r['longest_hand_only_hold_samples'],last=r['samples'][-1]) for r in results],indent=2))
