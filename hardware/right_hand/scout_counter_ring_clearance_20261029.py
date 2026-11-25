"""Small ring clearance hypotheses; no physical device access."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from hand_motor_mapping import ticks_to_sim
from screen_loaded_repeat_20261026 import cached_contacts
from slow_thumb_seam_resolution_20261026 import classify
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/counter_ring_clearance';OUT.mkdir(exist_ok=True)
def interp(a,b):
    n=max(1,int(np.ceil(max(abs(b-a))/2)));return np.rint([a+(b-a)*i/n for i in range(1,n+1)]).astype(int)
def main():
    base=np.load(ROOT/'right_hand_rotation_20261029/counter_yaw_scaled_scout/both07/candidate.npz');opened=base['start'];lo,hi=limits()
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build(env,OUT/'scene.xml',book_projection_mm=-10.);correct_model(m,opened)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);reports=[]
    for motor,delta in ((14,-128),(14,-64),(14,64),(14,128),(16,64),(16,128),(16,256)):
        custom=opened.copy();custom[motor-1]+=delta;goals=base['goals'].copy();goals[:,motor-1]+=delta
        approach=interp(opened,custom);values=np.vstack([opened,approach,goals]);within=bool(np.all(values>=lo) and np.all(values<=hi) and max(abs(np.diff(values,axis=0)).ravel())<=2)
        name=f'RF{motor}_{delta:+d}';folder=OUT/name;folder.mkdir(exist_ok=True);path=folder/'candidate.npz'
        np.savez_compressed(path,start=custom,goals=goals,phases=base['phases'],empty_ring_approach=approach,original_open=opened,ring_motor=motor,ring_delta_ticks=delta)
        row=dict(name=name,ring_motor=motor,ring_delta_ticks=delta,native_and_two_tick_bounds=within,candidate=path.relative_to(ROOT).as_posix(),candidate_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        if within:
            d=mujoco.MjData(m);d.qpos[-7:]=[0,0,10,1,0,0,0];maxself=maxsupport=0.;first=None;pair=None
            for i,q in enumerate(values):
                d.qpos[:16]=ticks_to_sim(q);mujoco.mj_forward(m,d);su,sp=inspect(d);sp=classify(sp);maxself=max(maxself,max(sp.values(),default=0.));maxsupport=max(maxsupport,max(su.values(),default=0.))
                if maxself>1e-6 or maxsupport>.15:first=i;pair=max(sp,key=sp.get) if sp else None;break
            row.update(static_max_self_mm=maxself,static_max_support_mm=maxsupport,first_rejected_goal=first,worst_self_pair=pair,geometry_passed=first is None)
        else:row.update(geometry_passed=False)
        reports.append(row);print(json.dumps(row),flush=True)
        (OUT/'geometry_review.json').write_text(json.dumps(dict(cases=reports,devices_opened=False,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
