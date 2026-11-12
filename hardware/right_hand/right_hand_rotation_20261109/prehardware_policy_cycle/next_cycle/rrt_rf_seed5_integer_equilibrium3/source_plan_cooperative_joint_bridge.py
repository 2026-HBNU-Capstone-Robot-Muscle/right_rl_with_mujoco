"""RRT-Connect proposal, then unchanged1ms free-cube trajectory validation.

Collision planning holds the source cube pose only in a separate FK query.
Accepted motion evidence comes exclusively from subsequent free dynamics.
"""
import argparse
import time
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,motor,ticks_to_sim,BOUND,SIM_TO_MOTOR_IDS,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg
from inspect_endpoints import inspect
from replay_cooperative_orders import ramp
from convex_distance_check_20261104 import CheckedDistance


class Tree:
    def __init__(self,root,is_start):
        self.nodes=[root.copy()];self.parents=[-1];self.is_start=is_start
    def add(self,node,parent):
        self.nodes.append(node.copy());self.parents.append(parent);return len(self.nodes)-1
    def nearest(self,target):
        return int(np.argmin(np.sum((np.asarray(self.nodes)-target)**2,axis=1)))
    def path(self,index):
        out=[]
        while index>=0:out.append(self.nodes[index]);index=self.parents[index]
        return out[::-1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--seed',type=int,default=0);p.add_argument('--tag',default='nominal')
    p.add_argument('--iterations',type=int,default=5000);p.add_argument('--edge-ticks',type=int,default=4)
    p.add_argument('--preload',type=float,default=.3)
    p.add_argument('--proposal-family',default='clearance_rf')
    p.add_argument('--retain-index-contact',action='store_true')
    p.add_argument('--retain-opposing-contact',action='store_true',help='Keep index OR ring geometry in contact; actual force transfer requires free-dynamics validation')
    p.add_argument('--include-thumb',action='store_true')
    p.add_argument('--proposal-path',help='Explicit saved proposal or freely validated equilibrium endpoint')
    p.add_argument('--equilibrium-angles',action='store_true',help='Plan toward measured endpoint angles with source servo lag, not a pose reset')
    p.add_argument('--source',default='connected_sac_and_teacher/first_quarter_end_state.npz')
    a=p.parse_args()
    if a.retain_index_contact and a.retain_opposing_contact:raise ValueError('Choose one contact manifold')
    folder=OUT/'next_cycle'/f'rrt_rf_seed{a.seed}_{a.tag}';folder.mkdir(parents=True,exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed run')
    from pathlib import Path
    (folder/'source_plan_cooperative_joint_bridge.py').write_bytes(Path(__file__).read_bytes())
    m,_,_,_,provenance=assets('active');source=OUT/a.source
    z=dict(np.load(source,allow_pickle=False));proposal_path=OUT/a.proposal_path if a.proposal_path else OUT/f'next_cycle/{a.proposal_family}/proposal{a.seed}.npz'
    static=dict(np.load(proposal_path,allow_pickle=False));d=restore(m,z);g=Guard(m)
    cube=m.geom('cube').id;tip=m.geom('rf_tip').id;B0=d.geom_xmat[cube].reshape(3,3).copy();C0=d.geom_xpos[cube].copy()
    start=motor(d.ctrl);lag=d.ctrl-d.qpos[:16];sid=np.array(SIM_TO_MOTOR_IDS)-1;free=sid[1:16] if a.include_thumb else sid[1:12]
    query=restore(m,z);query.qpos[:]=static['qpos'];query.ctrl[:]=query.qpos[:16]+lag;query.qvel[:]=0;mujoco.mj_forward(m,query)
    contact=next(c for c in query.contact if {int(c.geom1),int(c.geom2)}=={cube,tip})
    normal=contact.frame[:3]*(1 if int(contact.geom1)==cube else -1);J=np.zeros((3,m.nv))
    mujoco.mj_jac(m,query,J,None,contact.pos,int(m.geom_bodyid[tip]))
    target_ctrl=static['qpos'][:16]+lag if a.equilibrium_angles else static['ctrl'].copy();closing=-J[:,:16].T@normal/m.actuator_gainprm[:,0]
    target_ctrl[8:12]+=a.preload*closing[8:12];end=motor(target_ctrl)
    nonfree=np.setdiff1d(np.arange(16),free)
    end[nonfree]=start[nonfree]
    low=np.ceil(g.lo[free]+2).astype(int);high=np.floor(g.hi[free]-2).astype(int)
    index_tip=m.geom('if_tip').id;ring_tip=m.geom('rf_tip').id;index_geometry=CheckedDistance(m,cube,[index_tip,ring_tip])
    index_free=np.array([int(np.flatnonzero(free==sid[j])[0]) for j in (1,2,3)])
    ring_free=np.array([int(np.flatnonzero(free==sid[j])[0]) for j in (8,9,10,11)])
    nquery=0;reject={};cache={};started=time.perf_counter();rng=np.random.default_rng(a.seed+310)
    def set_query(x):
        goal=start.copy();goal[free]=x
        query.qpos[:]=z['qpos'];query.qpos[:16]+=ticks_to_sim(goal)-ticks_to_sim(start)
        query.ctrl[:]=ticks_to_sim(goal);query.qvel[:]=0;mujoco.mj_forward(m,query)
    def index_gap(x,which=None):
        set_query(x);return index_geometry.gap(query,index_tip if which is None else which)*1000
    def project(x,near=None):
        if not(a.retain_index_contact or a.retain_opposing_contact):return x
        x=x.copy();lower=low.copy();upper=high.copy()
        if near is not None:lower=np.maximum(lower,near-64);upper=np.minimum(upper,near+64)
        for _ in range(12):
            which=index_tip
            if a.retain_opposing_contact and abs(index_gap(x,ring_tip)+.02)<abs(index_gap(x,index_tip)+.02):which=ring_tip
            chosen_free=ring_free if which==ring_tip else index_free
            value=index_gap(x,which)
            if abs(value+.02)<.08:return x
            grad=[]
            for j in chosen_free:
                plus=x.copy();minus=x.copy();plus[j]+=2;minus[j]-=2
                grad.append((index_gap(plus,which)-index_gap(minus,which))/4)
            grad=np.asarray(grad);length=float(grad@grad)
            if length<1e-12:break
            change=np.clip(-(value+.02)*grad/length,-16,16)
            x[chosen_free]+=np.rint(change).astype(int);x=np.maximum(lower,np.minimum(upper,x))
        return x
    def valid(x):
        nonlocal nquery
        key=tuple(int(v) for v in x)
        if key in cache:return cache[key]
        nquery+=1;goal=start.copy();goal[free]=x
        if np.any(goal<g.lo) or np.any(goal>g.hi):return False
        set_query(x)
        support,own=g.contacts(query);pen=max((max(0.,-float(c.dist)*1000) for c in query.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        why=[]
        if max(own.values(),default=0.)>1e-6:why.append('self')
        if max(support.values(),default=0.)>.15:why.append('support')
        if pen>1:why.append('cube')
        if a.retain_index_contact and abs(index_geometry.gap(query,index_tip)*1000)>.3:why.append('index_contact')
        if a.retain_opposing_contact and min(abs(index_geometry.gap(query,gid)*1000) for gid in (index_tip,ring_tip))>.3:why.append('opposing_contact')
        for reason in why:reject[reason]=reject.get(reason,0)+1
        cache[key]=not why;return not why
    def edge(x,y,step=None):
        n=max(1,int(np.ceil(np.max(np.abs(y-x))/(a.edge_ticks if step is None else step))))
        for k in range(1,n+1):
            if not valid(np.rint(x+(y-x)*k/n).astype(int)):return False
        return True
    def extend(tree,goal):
        i=tree.nearest(goal);node=tree.nodes[i];distance=np.max(abs(goal-node))
        if distance==0:return i,True
        nxt=np.rint(node+(goal-node)*min(1.,64/distance)).astype(int)
        if (a.retain_index_contact or a.retain_opposing_contact) and not valid(nxt):nxt=project(nxt,node)
        if np.array_equal(nxt,node):return None,False
        if not edge(node,nxt):return None,False
        j=tree.add(nxt,i);return j,bool(np.array_equal(nxt,goal))
    start_valid=valid(start[free]);end_valid=valid(end[free]);raw=None;iteration=-1
    if start_valid and not end_valid:
        # Quantized goal neighborhoods are proposals, never immediate commands.
        candidates=[]
        for _ in range(2500):
            x=end[free]+rng.integers(-4,5,len(free));x=np.maximum(low,np.minimum(high,x))
            if valid(x):candidates.append(x.copy())
            if len(candidates)>=12:break
        if candidates:end[free]=min(candidates,key=lambda x:np.sum((x-end[free])**2));end_valid=True
    if start_valid and end_valid:
        A=Tree(start[free],True);B=Tree(end[free],False)
        if edge(start[free],end[free]):raw=[start[free],end[free]]
        for iteration in range(a.iterations):
            if raw is not None:break
            if time.perf_counter()-started>180:break
            if rng.random()<.18:sample=B.nodes[0]
            elif (a.retain_index_contact or a.retain_opposing_contact) and rng.random()<.7:
                anchor=A.nodes[int(rng.integers(len(A.nodes)))];sample=np.maximum(low,np.minimum(high,anchor+rng.integers(-320,321,len(free))))
                sample=project(sample)
            else:sample=rng.integers(low,high+1)
            i,_=extend(A,sample)
            if i is not None:
                for _ in range(60):
                    j,reached=extend(B,A.nodes[i])
                    if j is None:break
                    if reached:
                        pa=A.path(i);pb=B.path(j)
                        raw=pa+pb[-2::-1] if A.is_start else pb+pa[-2::-1]
                        break
            if iteration%200==0:print(json.dumps(dict(iteration=iteration,nodes=len(A.nodes)+len(B.nodes),queries=nquery)),flush=True)
            A,B=B,A
    planned=[];dynamic=None
    if raw is not None:
        for _ in range(160):
            if len(raw)<3:break
            i,j=sorted(rng.choice(len(raw),2,replace=False))
            if j>i+1 and edge(raw[i],raw[j],2):raw=raw[:i+1]+raw[j:]
        for x,y in zip(raw[:-1],raw[1:]):
            if not edge(x,y,2):raise RuntimeError('Final2tick collision check rejected the proposed bridge')
            first=start.copy();first[free]=x;last=start.copy();last[free]=y
            planned.extend(ramp(first,last))
        planned.extend(end.copy() for _ in range(134))
        if np.max(abs(np.diff(np.vstack([start,np.asarray(planned)]),axis=0)))>2:
            raise AssertionError('Planned path violates unchanged two-tick goal spacing')
        np.savez_compressed(folder/'research_goals.npz',start=start,goals=planned,waypoints=raw)
        data=restore(m,z);checker=Guard(m);held=good=0;stop=None;hold_B=hold_C=None;drift=shift=0.
        for k,goal in enumerate(planned):
            data.ctrl[:]=ticks_to_sim(goal)
            for _ in range(75):
                mujoco.mj_step(m,data);stop=checker.check(data)
                if np.linalg.norm(data.qpos[-7:-4]-z['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if k>=len(planned)-54:
                    force={}
                    for ci,c in enumerate(data.contact):
                        x,y=int(c.geom1),int(c.geom2)
                        if cube not in (x,y):continue
                        name=m.geom(y if x==cube else x).name
                        if name in ('if_bs_collision_1','rf_tip','th_tip'):
                            f=np.zeros(6);mujoco.mj_contactForce(m,data,ci,f);force[name]=force.get(name,0.)+float(f[0])
                    held+=1;good+=all(force.get(n,0.)>.2 for n in ('if_bs_collision_1','rf_tip','th_tip'))
                    BB=data.geom_xmat[cube].reshape(3,3);CC=data.geom_xpos[cube]
                    if hold_B is None:hold_B=BB.copy();hold_C=CC.copy()
                    drift=max(drift,orientation_error_deg(BB,hold_B));shift=max(shift,float(np.linalg.norm(CC-hold_C)*1000))
                if stop:break
            if (k+1)%250==0 or stop:print(json.dumps(dict(dynamic_goals=k+1,stop=stop)),flush=True)
            if stop:break
        np.savez_compressed(folder/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
                            up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
        angle_error=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),B0)
        center=float(np.linalg.norm(data.geom_xpos[cube]-C0)*1000)
        dynamic=dict(goals_completed=k+1,planned_goals=len(planned),stop=stop,worst=checker.worst,
            endpoint=inspect(m,data,z['up'],z['initial_cube_rotation']),hold_samples=held,
            third_support_fraction=good/max(1,held),orientation_drift_deg=angle_error,center_error_mm=center,
            hold_orientation_drift_deg=drift,hold_center_drift_mm=shift,
            temporary_third_support_passed=not stop and held==4050 and good/4050>.8 and angle_error<5 and center<2 and drift<3 and shift<2,
            actual_cube_dynamics_never_reset_after_source=True)
    save_json(folder/'review.json',dict(status='collision_planned_cooperative_support_bridge',seed=a.seed,
        source_sha256=sha(source),proposal_sha256=sha(proposal_path),model_provenance=provenance,
        start_valid=start_valid,end_valid=end_valid,iterations=iteration+1,queries=nquery,collision_rejections=reject,
        collision_plan_found=raw is not None,static_cube_used_only_for_planning=True,dynamic=dynamic,
        existing_index_contact_retained_during_static_plan=a.retain_index_contact,
        index_or_ring_contact_retained_during_static_plan=a.retain_opposing_contact,
        thumb_participates_in_planning=a.include_thumb,equilibrium_endpoint_angles_used=a.equilibrium_angles,
        proposal_path=str(proposal_path.relative_to(OUT)),nonfree_command_targets_preserved=True,
        source_state_path=str(source.relative_to(OUT)),
        devices_opened=0,hardware_execution_allowed=False,second90_rotation_passed=False,
        elapsed_seconds=time.perf_counter()-started))
    print(json.dumps(dict(plan_found=raw is not None,dynamic=dynamic,seconds=time.perf_counter()-started)),flush=True)


if __name__=='__main__':main()
