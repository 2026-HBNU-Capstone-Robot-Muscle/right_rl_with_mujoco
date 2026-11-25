"""OFF-only plan: bounded support separation -> collision-free open/grasp route.

This produces reviewable geometry and a queue, never hardware permission.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json,time
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from loaded_quarter_model_policy_20261025 import correct_model
from book_support_contact_scope import derive_scope,allowed_initial_pairs
from book_passive_support_reentry_policy import ROOT_ESCAPE_GOALS,ROOT_ESCAPE_SECONDS
ROOT=Path(__file__).resolve().parent

def interpolate(a,b,spacing=2):
    n=max(1,int(np.ceil(np.max(abs(np.asarray(b)-a))/spacing)))
    return np.rint(a+(b-a)*np.arange(1,n+1)[:,None]/n).astype(int)

def main():
    latest=json.loads((ROOT/'right_hand_rotation_20261026/fresh_empty_start_latest.json').read_bytes());folder=ROOT/latest['folder'];out=folder/'recovery_plan_v2_ring_first';out.mkdir(exist_ok=True);raw=np.asarray(latest['position_ticks']);lo,hi=limits();assert latest['stable_endpoints'] and np.all(raw>=lo) and np.all(raw<=hi)
    seedfolder=ROOT/'right_hand_rotation_20261026/legacy_stage4_ring_uncurl_other_v1';seed=json.loads((seedfolder/'seed.json').read_bytes());target=np.asarray(seed['commanded_hand_ticks']);grasp=np.load(seedfolder/'rrt_thumb_pair3_path.npz')['goals']
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,out/'scene.xml');range_policy=correct_model(m,raw);m.geom_contype[m.geom_bodyid==m.body('cube').id]=0;m.geom_conaffinity[m.geom_bodyid==m.body('cube').id]=0;d=mujoco.MjData(m);hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    obstacles={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};allowed=allowed_initial_pairs(derive_scope(m));queries=0
    def contacts(q):
        nonlocal queries
        queries+=1;d.qpos[:16]=ticks_to_sim(q);d.qvel[:]=0;mujoco.mj_forward(m,d);self_pairs={};support={}
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);dep=max(0.,-float(c.dist)*1000)
            if dep<=1e-9:continue
            ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand;key='|'.join(sorted((m.geom(a).name,m.geom(b).name)))
            if ha and hb:self_pairs[key]=max(self_pairs.get(key,0.),dep)
            if (a in obstacles and hb) or (b in obstacles and ha):support[key]=max(support.get(key,0.),dep)
        return self_pairs,support
    initial_self,initial_support=contacts(raw);ring=raw.copy();ring[14]-=80;ring_path=interpolate(raw,ring);ring_clear=True
    for q in ring_path:
        sp,su=contacts(q)
        if sp or any(v>initial_support.get(k,0.)+1e-6 for k,v in su.items()):ring_clear=False;break
    root=ring.copy();root[[7,11]]=np.maximum(root[[7,11]],2472);root_path=interpolate(ring,root);previous=initial_support;root_log=[];release_ok=ring_clear and not initial_self and set(initial_support)<=allowed and max(initial_support.values(),default=0.)<=4;first_clear=None
    for i,q in enumerate(root_path):
        sp,su=contacts(q);new={p:v for p,v in su.items() if p not in initial_support and v>1e-6};increase={p:v-previous.get(p,0.) for p,v in su.items() if v>previous.get(p,0.)+1e-6};ok=not sp and not new and not increase
        root_log.append(dict(sample=i+1,max_support_mm=max(su.values(),default=0.),self_pairs=sp,new_pairs=new,depth_increase_mm=increase))
        if not ok:release_ok=False;break
        if first_clear is None and max(su.values(),default=0.)<=.15:first_clear=i+1
        previous=su
    report=dict(status='fresh_empty_recovery_geometry_diagnostic',source=latest,initial_self_contacts=initial_self,initial_support_contacts=initial_support,initial_support_scope_verified=set(initial_support)<=allowed,early_ring_release_samples=len(ring_path),early_ring_release_clear=ring_clear,root_escape_samples=len(root_path),root_escape_first_clear_sample=first_clear,existing_root_escape_goal_bound=ROOT_ESCAPE_GOALS,existing_root_escape_seconds=ROOT_ESCAPE_SECONDS,root_escape_monotonic_and_no_new_contacts=release_ok,root_escape_geometry_log=root_log,range_policy=range_policy,hardware_execution_allowed=False,real_devices_opened=False)
    report['root_escape_within_existing_bounds']=bool(release_ok and first_clear is not None and first_clear<=ROOT_ESCAPE_GOALS and first_clear*.075<=ROOT_ESCAPE_SECONDS)
    if not report['root_escape_within_existing_bounds']:
        report['status']='support_release_needs_a_different_route';(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k not in ('root_escape_geometry_log','source','range_policy')},indent=2),flush=True);return
    def valid(x):
        x=np.rint(x).astype(int)
        if np.any(x<lo) or np.any(x>hi):return False
        a,b=contacts(x);return max(a.values(),default=0.)<=1e-6 and max(b.values(),default=0.)<=.15
    def edge(a,b,spacing=4):return all(valid(q) for q in interpolate(a,b,spacing))
    assert valid(root) and valid(target);rng=np.random.default_rng(20261027);start=time.perf_counter();A=[root.astype(float)];B=[target.astype(float)];AP=[-1];BP=[-1];orientation=True;solution=None
    def extend(nodes,parents,t):
        k=int(np.argmin(np.linalg.norm(np.asarray(nodes)-t,axis=1)));a=nodes[k];scale=min(1.,100/max(1e-9,np.max(abs(t-a))));b=a+(t-a)*scale
        if not edge(a,b):return None,False
        nodes.append(b);parents.append(k);return len(nodes)-1,bool(scale==1.)
    def branch(nodes,parents,k):
        path=[]
        while k>=0:path.append(nodes[k]);k=parents[k]
        return list(reversed(path))
    for iteration in range(8000):
        if time.perf_counter()-start>100:break
        t=B[0] if rng.random()<.3 else np.clip(rng.uniform(np.minimum(root,target)-350,np.maximum(root,target)+350),lo,hi)
        k,_=extend(A,AP,t)
        if k is not None:
            for _ in range(60):
                j,reached=extend(B,BP,A[k])
                if j is None:break
                if reached:
                    p=branch(A,AP,k)+list(reversed(branch(B,BP,j)))[1:];solution=p if orientation else list(reversed(p));break
            if solution is not None:break
        A,B=B,A;AP,BP=BP,AP;orientation=not orientation
    path=[]
    if solution is not None:
        for _ in range(120):
            if len(solution)<3:break
            a,b=sorted(rng.integers(0,len(solution),2))
            if b>a+1 and edge(solution[a],solution[b]):solution=solution[:a+1]+solution[b:]
        for a,b in zip(solution,solution[1:]):path.extend(interpolate(np.rint(a).astype(int),np.rint(b).astype(int)))
        assert all(valid(q) for q in path) and all(valid(q) for q in grasp)
        all_goals=np.vstack([ring_path,root_path,path,grasp,np.repeat(grasp[-1][None,:],27,axis=0)]);assert np.max(abs(np.diff(np.vstack([raw,all_goals]),axis=0)))<=2;assert np.all(all_goals>=lo) and np.all(all_goals<=hi)
        phases=np.array(['early_ring_release']*len(ring_path)+['initial_support_release']*len(root_path)+['empty_recovery_to_open']*len(path)+['empty_new_grasp']*len(grasp)+['empty_final_hold']*27)
        np.savez_compressed(out/'path.npz',start=raw,goals=all_goals,phases=phases,root_end=root,open_goal=target,grasp_goal=grasp[-1]);report.update(goals=len(all_goals),nominal_seconds=.075*len(all_goals),queue_sha256=hashlib.sha256(all_goals.astype('<i4').tobytes()).hexdigest(),end_goal_ticks=grasp[-1].tolist())
    report.update(solution_found=solution is not None,recovery_goals=len(path),queries=queries,planning_wall_seconds=time.perf_counter()-start,offset_and_dynamic_sensitivity_not_checked=True,model_manifest=manifest);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k not in ('root_escape_geometry_log','source','range_policy','model_manifest')},indent=2),flush=True)
if __name__=='__main__':main()
