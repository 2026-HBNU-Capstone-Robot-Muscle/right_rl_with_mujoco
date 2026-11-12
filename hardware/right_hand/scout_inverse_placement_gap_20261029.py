"""Find a physically accessible placement gap around the inverse ready pose.

Static hypotheses only. No force-fit, collision exemptions, or devices.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import itertools,json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from sac_cube_model import set_cube_physics
from screen_loaded_repeat_20261026 import cached_contacts
from slow_thumb_seam_resolution_20261026 import classify

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261029/inverse_placement_gap'
def main():
    OUT.mkdir(exist_ok=True)
    ready=np.load(ROOT/'right_hand_rotation_20261029/prepositioned_reverse_cube/candidate.npz')['start'].astype(int)
    cubeq=np.array(json.loads((ROOT/'right_hand_rotation_20261029/prepositioned_reverse_cube/held_cube_seed.json').read_bytes())['cube_qpos'])
    up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build(env,OUT/'scene.xml',book_projection_mm=-10.);correct_model(m,ready);set_cube_physics(m,.055,.1)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);cube=m.geom('cube').id;d=mujoco.MjData(m);lo,hi=limits()
    def probe(q,height=0.):
        d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:]=cubeq;d.qpos[-7:-4]+=up*height;mujoco.mj_forward(m,d)
        su,sp=inspect(d);sp=classify(sp);depths={}
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    name=m.geom(other).name;depths[name]=max(depths.get(name,0.),max(0.,-float(c.dist)*1000))
        return max(depths.values(),default=0.),max(sp.values(),default=0.),max(su.values(),default=0.),depths
    depth,_,_,original_details=probe(ready)
    print('original ready cube depth',depth,sorted(original_details.items(),key=lambda a:-a[1])[:6],flush=True)
    derivatives=[]
    for i in (0,1,2,8,9,10,11):
      for delta in (-32,32):
        q=ready.copy();q[i]+=delta
        if np.any(q<lo) or np.any(q>hi):continue
        dp,se,su,_=probe(q)
        derivatives.append(dict(motor_id=i+1,delta_ticks=delta,cube_depth_mm=dp,self_mm=se,support_mm=su))
    print('local changes',json.dumps(derivatives),flush=True)
    rows=[]
    base_cubeq=cubeq.copy()
    for height_offset,thumb_mcp,middle_pip,thumb_tip in itertools.product((0.,.0005,.001,.002),(0,-32,-64,-96,-128),(0,-16,-32,-64),(0,-32,-64,-96)):
        cubeq[:]=base_cubeq;cubeq[:3]+=up*height_offset
        q=ready.copy();q[1]+=thumb_mcp;q[10]+=middle_pip;q[0]+=thumb_tip
        if np.any(q<lo) or np.any(q>hi):continue
        dp,se,su,details=probe(q)
        if se>1e-6 or su>.15 or dp>.1:continue
        worst=dp
        for height in np.arange(.04,-.0001,-.001):
            dp,se,su,_=probe(q,float(height));worst=max(worst,dp)
            if dp>1. or se>1e-6 or su>.15:break
        else:
            rows.append(dict(placement_goal=q.tolist(),delta_ticks=(q-ready).tolist(),
                change_l1_ticks=int(sum(abs(q-ready))),initial_cube_depth_mm=max(details.values(),default=0.),
                insertion_max_cube_depth_mm=worst,cube_qpos=cubeq.tolist(),
                seed_height_offset_mm=height_offset*1000))
    rows.sort(key=lambda r:(r['change_l1_ticks'],r['seed_height_offset_mm']))
    report=dict(original_ready=ready.tolist(),original_cube_depth_mm=depth,
        original_contact_depths_mm=original_details,local_changes=derivatives,
        passed_candidates=rows,devices_opened=False,hardware_execution_allowed=False,
        initial_gap_threshold_mm=.1,vertical_insertion_depth_limit_mm=1.,
        physical_placement_accessibility_verified=False)
    (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('accessible static candidates',len(rows),'best',json.dumps(rows[:3]),flush=True)

if __name__=='__main__':main()
