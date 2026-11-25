"""Find small native-range changes to original lateral poses, offline only."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,itertools,json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_thumb_rotation_plan_20261026 import motion_goals
from loaded_quarter_reentry_20261025 import limits
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/yaw_original_clearance_search'
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--raised-start',action='store_true');parser.add_argument('--after-flip',action='store_true');parser.add_argument('--free-ring',action='store_true');args=parser.parse_args();output=OUT/(('after_flip_free_ring' if args.free_ring else 'after_flip') if args.after_flip else ('raised_start_free_ring' if args.free_ring else ('raised_start' if args.raised_start else '.')))
    output.mkdir(parents=True,exist_ok=True);saved=json.loads((ROOT/'loaded_repeat_20261026_170932_163649/result.json').read_bytes());sample=next(s for s in saved['samples'] if s['phase']=='stage_6_loaded_quarter_reference')
    base=motion_goals()[0].copy();residual=np.asarray(sample['actual'])-base;original=json.loads((ROOT/'sequence_flip_poses.json').read_bytes())['poses']
    after_seed=json.loads((ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1/seed.json').read_bytes()) if args.after_flip else None
    if after_seed is not None:residual=np.asarray(after_seed['response_residual_ticks'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/'scene.xml');d=mujoco.MjData(m);lo,hi=limits();hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};out=[]
    variants=([(True,not args.free_ring,d1,d2,floor) for d1,d2,floor in itertools.product((-128,0,128),(-256,-128,-64,0,64,128,256),(2250,2472,2650))]
        if args.raised_start else [(f,r,d1,d2,None) for f,r,d1,d2 in itertools.product((False,True),(False,True),(-16,0,16),(-16,0,16))])
    for flat,ring_fixed,delta1,delta2,root_floor in variants:
        base=np.asarray(after_seed['commanded_hand_ticks']) if after_seed is not None else motion_goals()[0].copy()
        if root_floor is not None and after_seed is None:base[[7,11]]=root_floor
        goals=[base.copy()];previous=base.copy()
        for v in original[1 if after_seed is not None else 3:]:
            target=np.asarray(v['raw_ticks']).copy();target[0]+=delta1;target[1]+=delta2
            if flat:target[[7,11]]=np.maximum(target[[7,11]],[root_floor,root_floor] if root_floor is not None else base[[7,11]])
            if ring_fixed:target[12:]=base[12:]
            n=max(1,int(np.ceil(np.max(abs(target-previous))/2)));goals.extend(np.rint(previous+(target-previous)*i/n).astype(int) for i in range(1,n+1));previous=target
        goals=np.asarray(goals)
        if not (np.all(goals>=lo) and np.all(goals<=hi) and np.max(abs(np.diff(goals,axis=0)))<=2):continue
        sp=sup=0.;bad=None
        for index,g in enumerate(goals):
            d.qpos[:16]=ticks_to_sim(g+residual);d.qpos[-7:-4]=[0,0,10];mujoco.mj_forward(m,d)
            for c in d.contact:
                a,b=int(c.geom1),int(c.geom2);dep=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                if ha and hb:sp=max(sp,dep)
                if (a in support and hb) or (b in support and ha):sup=max(sup,dep)
                if dep>0 and ((ha and hb and dep>.01) or (((a in support and hb) or (b in support and ha)) and dep>.15)):
                    bad=dict(index=index,pair=[m.geom(a).name,m.geom(b).name],depth_mm=dep);break
            if bad:break
        row=dict(flat_roots=flat,ring_fixed=ring_fixed,root_floor=root_floor,thumb1_delta_ticks=delta1,thumb2_delta_ticks=delta2,goals=len(goals),maximum_self_overlap_mm=sp,maximum_support_overlap_mm=sup,first_rejected=bad)
        if bad is None:
            name=f'floor{root_floor}_flat{int(flat)}_rf{int(ring_fixed)}_th1{delta1}_th2{delta2}.npz';np.savez_compressed(output/name,goals=goals,response_residual_ticks=residual);row['path']=name
        out.append(row);print('CLEARANCE',json.dumps(row),flush=True)
    (output/'review.json').write_text(json.dumps(dict(cases=out,model=manifest,actual_response_unidentified=True,hardware_execution_allowed=False,real_devices_opened=False),indent=2)+'\n',encoding='utf-8')
    print('SUMMARY',dict(cases=len(out),clear=sum(v['first_rejected'] is None for v in out)),flush=True)
if __name__=='__main__':main()
