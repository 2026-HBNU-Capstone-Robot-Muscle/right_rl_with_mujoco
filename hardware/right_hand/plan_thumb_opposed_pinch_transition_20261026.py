"""Joint-space RRT for after-flip thumb/finger regrasp. Model-only planning."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,time
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1'
def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('--pair',type=int,default=0);parser.add_argument('--seed-folder');args=parser.parse_args()
    if args.seed_folder:OUT=(ROOT/args.seed_folder).resolve();OUT.relative_to(ROOT)
    seed=json.loads((OUT/'seed.json').read_bytes());base=np.asarray(seed['commanded_hand_ticks']);residual=np.asarray(seed['response_residual_ticks']);state=np.asarray(seed['settled_cube_qpos']);pairs=[v for v in json.loads((OUT/'thumb_opposed_pinch_endpoints.json').read_bytes())['pairs'] if v['endpoint_promising']];pair=pairs[args.pair];end=np.asarray(pair['goal_ticks']);ids=np.r_[np.array([3,2,1,0]),np.array([7,5,6,4] if pair['finger']=='if' else [11,9,10,8])];low,high=limits()
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/f'rrt_thumb_pair{args.pair}_scene.xml');set_cube_physics(m,.055,.1);d=mujoco.MjData(m);hand={m.body('palm').id};cube=m.geom('cube').id
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};queries=0;start=time.perf_counter();rng=np.random.default_rng(2630+args.pair)
    def valid(x):
        nonlocal queries
        queries+=1;q=base.copy();q[ids]=np.rint(x).astype(int);d.qpos[:16]=ticks_to_sim(q+residual);d.qpos[-7:]=state;d.qvel[:]=0;mujoco.mj_forward(m,d)
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand;dep=-float(c.dist)*1000
            if ha and hb and dep>.01:return False
            if ((a in support and hb) or (b in support and ha)) and dep>.15:return False
        return True
    def edge(a,b,spacing=4):
        n=max(1,int(np.ceil(np.max(abs(b-a))/spacing)))
        return all(valid(a+(b-a)*i/n) for i in range(1,n+1))
    A=[base[ids].astype(float)];AP=[-1];B=[end[ids].astype(float)];BP=[-1];solution=None
    if not valid(A[0]) or not valid(B[0]):raise RuntimeError('Invalid endpoint')
    def extend(nodes,parents,target):
        k=int(np.argmin(np.linalg.norm(np.asarray(nodes)-target,axis=1)));a=nodes[k];scale=min(1.,80/max(1e-9,np.max(abs(target-a))));b=a+(target-a)*scale
        if not edge(a,b):return None,False
        nodes.append(b);parents.append(k);return len(nodes)-1,bool(scale==1.)
    def branch(nodes,parents,k):
        path=[]
        while k>=0:path.append(nodes[k]);k=parents[k]
        return list(reversed(path))
    original_start=A[0].copy();orientation=True
    for iteration in range(6000):
        if time.perf_counter()-start>75:break
        target=B[0] if rng.random()<.25 else (np.clip(rng.uniform(np.minimum(base[ids],end[ids])-500,np.maximum(base[ids],end[ids])+500),low[ids],high[ids]) if rng.random()<.8 else rng.uniform(low[ids],high[ids]))
        k,_=extend(A,AP,target)
        if k is not None:
            for _ in range(50):
                j,reached=extend(B,BP,A[k])
                if j is None:break
                if reached:
                    path=branch(A,AP,k)+list(reversed(branch(B,BP,j)))[1:];solution=path if orientation else list(reversed(path));break
            if solution is not None:break
        A,B=B,A;AP,BP=BP,AP;orientation=not orientation
    goals=[]
    if solution is not None:
        assert np.max(abs(solution[0]-original_start))<1e-8
        # Remove detours only if all interpolated poses remain clear.
        for _ in range(100):
            if len(solution)<3:break
            a,b=sorted(rng.integers(0,len(solution),2))
            if b>a+1 and edge(solution[a],solution[b]):solution=solution[:a+1]+solution[b:]
        previous=base.copy()
        for waypoint in solution[1:]:
            goal=base.copy();goal[ids]=np.rint(waypoint).astype(int);n=max(1,int(np.ceil(np.max(abs(goal-previous))/2)))
            goals.extend(np.rint(previous+(goal-previous)*i/n).astype(int) for i in range(1,n+1));previous=goal
        assert all(valid(g[ids]) for g in goals)
        if goals:assert np.max(abs(np.diff(np.vstack([base,np.asarray(goals)]),axis=0)))<=2
        np.savez_compressed(OUT/f'rrt_thumb_pair{args.pair}_path.npz',goals=np.asarray(goals),start=base,end=end,waypoints=np.asarray(solution))
    report=dict(status='joint_space_thumb_opposed_transition_screen',pair=args.pair,finger=pair['finger'],solution_found=solution is not None,iterations=iteration+1,queries=queries,elapsed_wall_seconds=time.perf_counter()-start,goals=len(goals),nominal_seconds=.075*len(goals),
        start_goal_ticks=base.tolist(),end_goal_ticks=end.tolist(),self_and_support_collision_checked=True,cube_dynamics_not_checked=True,
        initial_cube_not_registered=True,native_and_response_neighborhood_not_checked=True,hardware_execution_allowed=False,real_devices_opened=False)
    (OUT/f'rrt_thumb_pair{args.pair}_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
if __name__=='__main__':main()
