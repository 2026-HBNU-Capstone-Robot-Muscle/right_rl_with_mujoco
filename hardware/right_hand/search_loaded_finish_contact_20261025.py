"""Offline candidate search from the reproduced stall, never a motor program."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import itertools,json
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from sac_forward_geometry import target_quaternion
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261024/actual_loaded_review_171607'
def main():
    reference=json.loads((OUT/'actual_loaded_stalled_finish_candidates_v3.json').read_bytes())
    seed=next(c for c in reference['cases'] if c['offset_xy_mm']==[-4,0] and c['hypothetical_finish_extra_root_ticks']==0)
    seedpath=OUT/'loaded_measured_prior_model_extra0_dx-4_stalled_finish_v3.npz';stored=np.load(seedpath)['qpos'];seedq=stored[-1]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'finish_contact_search_scene.xml');set_cube_physics(m,.055,.1);m.dof_armature[:16]=1e6
    cube=m.geom('cube').id;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};target=target_quaternion(np.array(seed['settled_cube_qpos'][-4:]));step=float(m.opt.timestep)
    zeros=ticks_to_sim(np.zeros(16,dtype=int))
    def evaluate(extra,tip,cmc):
        a=np.zeros(16);a[0]=tip;a[3]=cmc;b=a.copy();b[[7,11]]=extra
        da=ticks_to_sim(a)-zeros;db=ticks_to_sim(b)-zeros
        na=max(1,int(np.ceil(max(abs(a))/2)));nb=max(1,int(np.ceil(extra/2)))
        points=np.vstack([seedq[:16],seedq[:16]+da,seedq[:16]+db,seedq[:16]+db]);grid=np.array([0,na*.05,(na+nb)*.05,(na+nb)*.05+3])
        ts=np.arange(0,grid[-1]+step/2,step);poses=np.column_stack([np.interp(ts,grid,points[:,j]) for j in range(16)]);vel=np.gradient(poses,step,axis=0)
        if np.any(poses<m.jnt_range[:16,0]-1e-6) or np.any(poses>m.jnt_range[:16,1]+1e-6):return dict(extra_roots=extra,thumb_tip=tip,thumb_cmc=cmc,range_rejected=True)
        d=mujoco.MjData(m);d.qpos[:]=seedq;d.qvel[:]=0;mujoco.mj_forward(m,d);maxself=maxsupport=gap=maxgap=0.;external=False;errors=[];endtouch=False;depth=0
        for i,t in enumerate(ts):
            d.qpos[:16]=poses[i];d.qvel[:16]=vel[i];d.ctrl[:]=poses[i];mujoco.mj_forward(m,d);touch=False
            for c in d.contact:
                x,y=int(c.geom1),int(c.geom2);hx,hy=int(m.geom_bodyid[x]) in hand,int(m.geom_bodyid[y]) in hand
                if c.dist<0 and hx and hy:maxself=max(maxself,-float(c.dist)*1000)
                if c.dist<0 and (x in support and hy or y in support and hx):maxsupport=max(maxsupport,-float(c.dist)*1000)
                if cube in (x,y):
                    other=y if x==cube else x
                    if int(m.geom_bodyid[other]) in hand:
                        touch|=c.dist<.0003;depth=max(depth,-float(c.dist)*1000)
                    elif c.dist<.0003 and (other in support or m.geom(other).name=='floor'):external=True
            gap=0 if touch else gap+step;maxgap=max(maxgap,gap);endtouch=touch
            errors.append(float(np.degrees(2*np.arccos(np.clip(abs(d.qpos[-4:]@target)/np.linalg.norm(d.qpos[-4:]),0,1)))))
            if maxself>.01:break
            mujoco.mj_step(m,d)
        holdmax=float(max(errors[max(0,len(errors)-round(3/step)):]))
        row=dict(extra_roots=extra,thumb_tip=tip,thumb_cmc=cmc,final_error_deg=errors[-1],hold_error_deg=holdmax,
            self_overlap_mm=maxself,support_overlap_mm=maxsupport,cube_overlap_mm=depth,max_no_hand_contact_s=maxgap,external_cube_contact=external,
            hand_contact_at_end=bool(endtouch),source_frictions=m.geom_friction[cube].tolist(),native_ranges_preserved=True,
            candidate_accepted=bool(len(errors)==len(ts) and errors[-1]<=15 and holdmax<=20 and maxself<=1e-6 and maxsupport<=.15 and maxgap<=.1 and not external and endtouch))
        return row
    results=[]
    for a,b,c in itertools.product((128,192,256),(64,128,192),(-64,0,64)):
        row=evaluate(a,b,c);results.append(row)
        if row.get('candidate_accepted'):print('PASS',json.dumps(row),flush=True)
    accepted=[r for r in results if r.get('candidate_accepted')]
    report=dict(source_stall='actual_loaded_stalled_finish_candidates_v3.json',model=manifest,cases=results,accepted=accepted,
        only_one_unregistered_starting_state=True,robust_loaded_execution_ready=False,hardware_execution_allowed=False,hardware_commands_sent=0)
    (OUT/'finish_contact_search.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('SUMMARY',json.dumps(dict(cases=len(results),accepted=len(accepted),best=sorted([r for r in results if 'final_error_deg' in r],key=lambda r:r['final_error_deg'])[:3])),flush=True)
if __name__=='__main__':main()
