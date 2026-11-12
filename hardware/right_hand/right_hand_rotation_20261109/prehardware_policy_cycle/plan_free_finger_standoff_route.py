"""Plan a declared finger's route to an outward waypoint and replay dynamics."""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha,SIM_TO_MOTOR_IDS
from cube_task_frame_20261105 import orientation_error_deg
from whole_hand_support import sample_model
from directed_contact_state import tip_yaw_contact


def main():
    p=argparse.ArgumentParser();p.add_argument('--tag',default='robust_mu025_free_MF_standoff_route_v1');p.add_argument('--seed',type=int,default=20261111);p.add_argument('--iterations',type=int,default=1600);p.add_argument('--waypoint-study');p.add_argument('--gap',type=float,choices=(3.,5.,8.),default=8.);p.add_argument('--clearance-mm',type=float,default=1.);p.add_argument('--allow-proximal-support',action='store_true');p.add_argument('--account-for-static-lag',action='store_true');p.add_argument('--source-state');a=p.parse_args()
    if not .25<=a.clearance_mm<=1.:raise ValueError('Positive noncontact route clearance required')
    if not a.tag.replace('_','').isalnum() or not 1<=a.iterations<=4000:raise ValueError('Bounded local planning study required')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/a.tag;source=base/'robust_mu025_reposition_MF_reset_gait_v1/pip-30_dip15_mcp10/end_state.npz';goal_file=base/'robust_mu025_directed_standoff_plan_v2/gap8mm/waypoint_reference_only.npz'
    finger='mf'
    if a.waypoint_study:
        study=(OUT/a.waypoint_study).resolve()
        if not study.is_relative_to(OUT.resolve()):raise ValueError('Local waypoint study required')
        meta=json.loads((study/'review.json').read_text());assert meta['plan_only'];finger=meta['only_declared_finger_commands_changed'];assert finger in ('if','mf','rf','th')
        source=OUT/meta['source'];assert sha(source)==meta['source_sha256']
        row=next(r for r in meta['rows'] if r['distance_mm']==a.gap);assert row['static_waypoint_valid']
        goal_file=OUT/row['path']/'waypoint_reference_only.npz'
    waypoint_source=source
    if a.source_state:
        source=(OUT/a.source_state).resolve()
        if not source.is_relative_to(OUT.resolve()):raise ValueError('Local reached source required')
    if folder.exists():raise RuntimeError('Preserve prior free-finger route')
    folder.mkdir();(folder/'source_plan_free_finger_standoff_route.py').write_bytes(Path(__file__).read_bytes())
    m,initial,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=.25;z=dict(np.load(source,allow_pickle=False));query=restore(m,z);g=Guard(m);cube=m.geom('cube').id;tip=m.geom(finger+'_tip').id;start_index=4*('if','mf','rf','th').index(finger);active=np.arange(start_index,start_index+4);motor_active=np.sort(np.asarray(SIM_TO_MOTOR_IDS)[active]-1)
    start=motor(z['ctrl']);goal=motor(np.load(goal_file,allow_pickle=False)['ctrl']);assert np.array_equal(np.delete(start,motor_active),np.delete(goal,motor_active))
    static_lag=z['ctrl']-z['qpos'][:16] if a.account_for_static_lag else np.zeros(16)
    explicit={int(m.pair_geom2[i]) if int(m.pair_geom1[i])==cube else int(m.pair_geom1[i]) for i in range(m.npair) if cube in (int(m.pair_geom1[i]),int(m.pair_geom2[i]))}
    mf_geoms=[i for i in range(m.ngeom) if (m.body(int(m.geom_bodyid[i])).name or '').startswith(finger+'_') and ((m.geom_contype[i]&m.geom_conaffinity[cube]) or (m.geom_contype[cube]&m.geom_conaffinity[i]) or i in explicit)];mf_bodies={int(m.geom_bodyid[i]) for i in mf_geoms};cache={};rejections={};points=np.zeros(6);clearance_mm=a.clearance_mm
    clearance_geoms=[i for i in mf_geoms if not a.allow_proximal_support or m.geom(i).name not in (finger+'_bs_collision_1',finger+'_px_collision')]
    def valid(q):
        key=tuple(np.rint(q).astype(int))
        if key in cache:return cache[key]
        command=start.copy();command[motor_active]=key
        if np.any(command<g.lo) or np.any(command>g.hi):cache[key]=False;return False
        query.qpos[:]=z['qpos'];query.qpos[active]=ticks_to_sim(command)[active]-static_lag[active];query.ctrl[:]=z['ctrl'];query.ctrl[active]=ticks_to_sim(command)[active]
        if np.any(query.qpos[:16]<g.qlo) or np.any(query.qpos[:16]>g.qhi):cache[key]=False;return False
        mujoco.mj_forward(m,query)
        support,own=g.contacts(query)
        if max(own.values(),default=0.)>1e-6 or max(support.values(),default=0.)>.15:cache[key]=False;rejections['native_self_or_support']=rejections.get('native_self_or_support',0)+1;return False
        if any(cube in (int(c.geom1),int(c.geom2)) and c.dist<-.001 for c in query.contact):cache[key]=False;rejections['native_cube_penetration']=rejections.get('native_cube_penetration',0)+1;return False
        for gid in clearance_geoms:
            if mujoco.mj_geomDistance(m,query,cube,gid,.002,points)*1000<clearance_mm:
                key_name=f'{finger.upper()}_cube_clearance';cache[key]=False;rejections[key_name]=rejections.get(key_name,0)+1;return False
        cache[key]=True;return True
    def edge(x,y):
        n=max(1,int(np.ceil(np.max(abs(y-x))/4)))
        return all(valid(x+(y-x)*i/n) for i in range(1,n+1))
    summary=dict(source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),goal_reference=goal_file.relative_to(OUT).as_posix(),goal_reference_sha256=sha(goal_file),sliding_friction_hypothesis=.25,declared_finger=finger,model_provenance=provenance,planning_clearance_mm=clearance_mm,source_static_lag_compensation_requested=a.account_for_static_lag,static_lag_sim_rad=static_lag.tolist(),static_lag_is_proposal_approximation_not_dynamic_state_reset=True,cube_collision_geoms_checked=[m.geom(i).name for i in mf_geoms],additional_noncontact_clearance_geoms=[m.geom(i).name for i in clearance_geoms],proximal_cube_support_allowed_under_unchanged_native_guards=a.allow_proximal_support,model_collision_masks_and_explicit_pairs_unchanged=True,static_branches_do_not_reset_actual_state=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False)
    summary.update(original_waypoint_source=waypoint_source.relative_to(OUT).as_posix(),original_waypoint_source_sha256=sha(waypoint_source),explicit_reached_source_override=a.source_state is not None)
    first=start[motor_active].astype(float);last=goal[motor_active].astype(float);start_valid=valid(first);goal_valid=valid(last)
    if not start_valid or not goal_valid:
        endpoint_gaps={}
        for name,value in (('start',first),('goal',last)):
            command=start.copy();command[motor_active]=value;query.qpos[:]=z['qpos'];query.qpos[active]=ticks_to_sim(command)[active]-static_lag[active];mujoco.mj_forward(m,query)
            endpoint_gaps[name]={m.geom(gid).name:float(mujoco.mj_geomDistance(m,query,cube,gid,.02,points)*1000) for gid in mf_geoms}
        save_json(folder/'review.json',dict(summary,status='free_finger_waypoint_endpoint_rejected',start_valid=start_valid,goal_valid=goal_valid,endpoint_gaps_mm=endpoint_gaps,planning_rejections=rejections));print(json.dumps(dict(start_valid=start_valid,goal_valid=goal_valid,endpoint_gaps_mm=endpoint_gaps,rejections=rejections)),flush=True);return
    rng=np.random.default_rng(a.seed);trees=[dict(nodes=[first],parents=[-1],origin='start'),dict(nodes=[last],parents=[-1],origin='goal')];path=None;iterations=0
    def extend(tree,target):
        values=np.asarray(tree['nodes']);idx=int(np.argmin(np.sum(((values-target)/250)**2,axis=1)));before=values[idx];delta=target-before;length=np.max(abs(delta));q=np.rint(target if length<=40 else before+delta*40/length)
        if np.array_equal(q,before):return idx,np.array_equal(q,target)
        if not edge(before,q):return None,False
        tree['nodes'].append(q);tree['parents'].append(idx);return len(tree['nodes'])-1,np.array_equal(q,target)
    def chain(tree,index):
        values=[]
        while index>=0:values.append(tree['nodes'][index]);index=tree['parents'][index]
        return values[::-1]
    if edge(first,last):path=[first,last]
    for iteration in range(a.iterations if path is None else 0):
        iterations=iteration+1;left,right=trees;sample=right['nodes'][0] if rng.random()<.15 else rng.uniform(np.ceil(g.lo[motor_active])+2,np.floor(g.hi[motor_active])-2)
        index,_=extend(left,sample)
        if index is not None:
            target=left['nodes'][index]
            for _ in range(80):
                other,reached=extend(right,target)
                if other is None:break
                if reached:
                    path=chain(left,index)+chain(right,other)[::-1][1:]
                    if left['origin']=='goal':path=path[::-1]
                    break
        if path is not None:break
        trees=trees[::-1]
        if iterations%100==0:print(json.dumps(dict(iterations=iterations,queries=len(cache),nodes=sum(len(t['nodes']) for t in trees))),flush=True)
    if path is None:
        save_json(folder/'review.json',dict(summary,status='free_finger_route_not_found',iterations=iterations,queries=len(cache),planning_rejections=rejections));print('route_not_found',flush=True);return
    for _ in range(80):
        if len(path)<3:break
        i,j=sorted(rng.choice(len(path),2,replace=False))
        if j>i+1 and edge(path[i],path[j]):path=path[:i+1]+path[j:]
    np.savez_compressed(folder/'static_motor_waypoints.npz',**{finger.upper()+'_motor_ticks':path})
    d=restore(m,z);check=Guard(m);previous=start.copy();commands=[];C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();shift=drift=0.;stop=[];samples=supported=0
    for index,waypoint in enumerate(path[1:]+[path[-1]]):
        target=previous.copy();target[motor_active]=waypoint;before=previous.copy();count=max(1,int(np.ceil(np.max(abs(target-before))/2))) if index<len(path)-1 else 30
        for k in range(count):
            command=np.rint(before+(target-before)*(k+1)/count);assert np.max(abs(command-previous))<=2 and np.all(command>=check.lo) and np.all(command<=check.hi);d.ctrl[active]=ticks_to_sim(command)[active];previous=command.copy();commands.append(command.copy())
            for _ in range(75):
                mujoco.mj_step(m,d);stop=check.check(d);shift=max(shift,float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000));drift=max(drift,orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B0));samples+=1;supported+=sample_model(m,d,check.hand,cube)['supported_sample']
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if any(c.dist<0 and cube in (int(c.geom1),int(c.geom2)) and (int(c.geom2) if int(c.geom1)==cube else int(c.geom1)) in clearance_geoms for c in d.contact):stop=list(stop)+[f'unintended_{finger.upper()}_cube_contact_on_free_approach']
                if shift>2 or drift>3:stop=list(stop)+['standoff_disturbed_cube']
                if stop:break
            if stop:break
        print(json.dumps(dict(waypoint=index,goals=len(commands),stop=stop,shift_mm=shift,drift_deg=drift)),flush=True)
        if stop:break
    actual=tip_yaw_contact(m,d,cube,tip,initial['up']);np.savez_compressed(folder/'research_goals.npz',start=start,goals=commands);np.savez_compressed(folder/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    r=dict(summary,status=f'free_{finger.upper()}_standoff_route_planned_and_replayed',iterations=iterations,static_queries=len(cache),planning_rejections=rejections,waypoints=len(path),steps=len(commands),stop=stop,actual_tip_contact=actual,maximum_cube_shift_mm=shift,maximum_cube_drift_deg=drift,whole_hand_support_fraction=supported/max(1,samples),actual_standoff_reached=bool(not stop and actual['gap_mm']>.5 and actual['lever_mm']>3 and supported/max(1,samples)>.8),actual_state_resets_after_source=0,original_native_guards_and_joint_limits_preserved=True,worst=check.worst)
    save_json(folder/'review.json',r);print(json.dumps(r),flush=True)


if __name__=='__main__':main()
