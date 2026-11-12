"""Offline source-bound reentry and collision-free opening for a grasp trial."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json,time
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits,validate_path
from screen_loaded_repeat_20261026 import cached_contacts
from hand_motor_mapping import ticks_to_sim
from fresh_empty_grasp_plan_20261027 import interpolate
ROOT=Path(__file__).resolve().parent
def main():
    latest=json.loads((ROOT/'right_hand_rotation_20261027/grasp_off_latest.json').read_bytes());data=json.loads((ROOT/latest['result']).read_bytes());a=np.array(data['position_ticks']);out=ROOT/'right_hand_rotation_20261027/loaded_grasp_recovery';out.mkdir(exist_ok=True);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);models=[];lo,hi=limits();path=np.load(ROOT/'right_hand_rotation_20261026/fresh_empty_start_20261026_235258/recovery_plan_v2_ring_first/path.npz');target=path['open_goal'].copy();target[7]=max(target[7],2048)
    for front in (-15.,-10.,-5.):
        m,_=build(env,out/f'open_scene_{front:g}.xml',book_projection_mm=front);correct_model(m,a);m.geom_contype[m.geom_bodyid==m.body('cube').id]=0;m.geom_conaffinity[m.geom_bodyid==m.body('cube').id]=0;hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};models.append((m,mujoco.MjData(m),cached_contacts(m,hand,support)))
    calls=0
    def valid(q):
        nonlocal calls
        q=np.rint(q).astype(int);calls+=1
        if np.any(q<lo) or np.any(q>hi):return False
        for m,d,inspect in models:
            d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];d.qvel[:]=0;mujoco.mj_forward(m,d);su,sp=inspect(d)
            if max(su.values(),default=0.)>.15 or max(sp.values(),default=0.)>1e-6:return False
        return True
    def edge(x,y,step=6):return all(valid(q) for q in interpolate(x,y,step=2)[::max(1,step//2)]) and valid(y)
    entry=a.copy();entry[3]=1872;root=entry.copy();root[[7,11]]=2150;root[15]=2053;assert valid(root) and valid(target);start=time.perf_counter();rng=np.random.default_rng(271001);A=[root.astype(float)];B=[target.astype(float)];AP=[-1];BP=[-1];forward=True;solution=[root,target] if edge(root,target,2) else None
    def extend(nodes,parents,t):
        k=int(np.argmin(np.linalg.norm(np.asarray(nodes)-t,axis=1)));x=nodes[k];scale=min(1.,100/max(1e-9,np.max(abs(t-x))));y=x+(t-x)*scale
        if not edge(x,y):return None,False
        nodes.append(y);parents.append(k);return len(nodes)-1,scale==1.
    def branch(nodes,parents,k):
        p=[]
        while k>=0:p.append(nodes[k]);k=parents[k]
        return p[::-1]
    for iteration in range(8000):
        if solution is not None or time.perf_counter()-start>110:break
        sample=B[0] if rng.random()<.35 else np.clip(rng.uniform(np.minimum(root,target)-300,np.maximum(root,target)+300),lo,hi);i,_=extend(A,AP,sample)
        if i is not None:
            for _ in range(60):
                j,reached=extend(B,BP,A[i])
                if j is None:break
                if reached:
                    p=branch(A,AP,i)+branch(B,BP,j)[::-1][1:];solution=p if forward else p[::-1];break
        A,B=B,A;AP,BP=BP,AP;forward=not forward
    if solution is None:raise RuntimeError('No opening route found; no hardware permission')
    for _ in range(120):
        if len(solution)<3:break
        x,y=sorted(rng.integers(0,len(solution),2))
        if y>x+1 and edge(solution[x],solution[y],2):solution=solution[:x+1]+solution[y:]
    opened=[]
    for x,y in zip(solution,solution[1:]):opened.extend(interpolate(np.rint(x).astype(int),np.rint(y).astype(int)))
    assert all(valid(q) for q in opened);entry_rows=interpolate(a,entry);prefix=entry.copy();prefix[[7,11]]=np.minimum(root[[7,11]],entry[[7,11]]+120);prefix[15]=int(round(entry[15]+(root[15]-entry[15])*120/max(root[[7,11]]-entry[[7,11]])));roots=interpolate(entry,prefix,1.5)+interpolate(prefix,root,1);rows=[('entry',q) for q in entry_rows]+[('roots',q) for q in roots]+[('open',q) for q in opened];validate_path(a,rows,'entry');np.savez_compressed(out/'open_path.npz',start=a,entry_goal=entry,root_prefix=prefix,root_goal=root,open_goal=target,open_path=np.array(opened),goals=np.array([q for _,q in rows]),phases=np.array([p for p,_ in rows]));summary=dict(solution_found=True,source=latest,start_ticks=a.tolist(),entry_samples=len(entry_rows),root_samples=len(roots),opening_samples=len(opened),goals=len(rows),nominal_seconds=len(rows)*.075,wall_seconds=time.perf_counter()-start,queries=calls,open_ticks=target.tolist(),path_sha256=hashlib.sha256((out/'open_path.npz').read_bytes()).hexdigest(),hardware_motion_allowed=False);(out/'open_review.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8');print(json.dumps(summary),flush=True)
if __name__=='__main__':main()
