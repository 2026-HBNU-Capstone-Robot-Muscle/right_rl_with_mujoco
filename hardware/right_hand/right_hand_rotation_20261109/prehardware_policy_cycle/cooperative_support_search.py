"""Cooperative contact-mode proposals followed by free native dynamics.

Cube pose changes in IK are proposals only, never successful motion evidence.
Each dynamic comparison starts at the proven first-quarter endpoint and never
pins or replaces its cube state. Original physical settings are not edited.
"""
import argparse
import time
import json
import numpy as np
import mujoco
from scipy.optimize import least_squares
from native_cycle import OUT, ROOT, assets, restore, Guard, motor, ticks_to_sim, BOUND, TICK, SIM_TO_MOTOR_IDS, save_json, sha
from upper_layer_cylinder_clearance_20261105 import precompute, gaps
from cube_task_frame_20261105 import orientation_error_deg
from inspect_endpoints import inspect

SOURCE = 'connected_sac_and_teacher/first_quarter_end_state.npz'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--finger', choices=('rf','mf'), default='rf')
    p.add_argument('--mode', choices=('three_tips','replace_index'), default='replace_index')
    p.add_argument('--fits', type=int, default=24)
    a = p.parse_args()
    folder = OUT/'next_cycle'/f'cooperative_{a.finger}_{a.mode}'
    folder.mkdir(parents=True, exist_ok=True)
    if (folder/'review.json').exists():
        raise RuntimeError('Preserve completed search; use a separate run directory')
    m, _, _, _, provenance = assets('active')
    source = OUT/SOURCE; z = dict(np.load(source, allow_pickle=False))
    d = restore(m,z); guard = Guard(m); cube = m.geom('cube').id
    C0 = d.geom_xpos[cube].copy(); B = d.geom_xmat[cube].reshape(3,3).copy()
    q0=d.qpos[:16].copy(); lag=d.ctrl-q0; sid=np.array(SIM_TO_MOTOR_IDS)-1
    cache=precompute(m,guard.hand); vertices={gid:v for gid,v,_ in cache}
    target_geom=m.geom(a.finger+'_tip').id
    current_contacts={}
    for contact in d.contact:
        x,y=int(contact.geom1),int(contact.geom2)
        if cube in (x,y):
            other=y if x==cube else x; name=m.geom(other).name
            if name in ('if_tip','th_tip','if_bs_collision_1'):
                current_contacts.setdefault(name,[]).append(contact.pos.copy())
    required=('th_tip','if_bs_collision_1') if a.mode=='replace_index' else ('if_tip','th_tip','if_bs_collision_1')
    anchors=[]
    for name in required:
        gid=m.geom(name).id
        point=np.mean(current_contacts[name],axis=0)
        anchors.append((gid,d.geom_xmat[gid].reshape(3,3).T@(point-d.geom_xpos[gid]),B.T@(point-C0)))
    thumb_local=B.T@(np.mean(current_contacts['th_tip'],axis=0)-C0)
    dominant=max((0,2),key=lambda i:abs(thumb_local[i])); opposite=-np.sign(thumb_local[dominant])
    faces=[(dominant,opposite),(2 if dominant==0 else 0,1),(2 if dominant==0 else 0,-1),(dominant,-opposite)]
    targets=[]
    for axis,sign in faces:
        for height in (0.,14.,23.):
            local=np.zeros(3);local[axis]=sign*27.45;local[1]=height
            targets.append(local/1000)
    taught=q0.copy()
    if a.finger=='rf':taught[8:12]=(np.array([3053,2829,2961,3018])-2048)*TICK
    else:
        run=json.loads((ROOT/'middle_clearance_probe_20261106_230902_998326/result.json').read_text())
        taught[4:8]=(np.asarray(run['middle_probe_position_ticks'])[sid[4:8]]-2048)*TICK
    qlo=np.maximum(guard.qlo,(np.ceil(guard.lo[sid])+2-2048)*TICK-lag)
    qhi=np.minimum(guard.qhi,(np.floor(guard.hi[sid])-2-2048)*TICK-lag)
    lower=np.r_[qlo,[-20.]*3];upper=np.r_[qhi,[20.]*3]
    metadata=dict(source=SOURCE,source_sha256=sha(source),source_cube_center=C0.tolist(),
                  initial_cube_center=z['initial_cube_center'].tolist(),source_model=provenance,
                  source_tip_positions={name:d.geom_xpos[m.geom(name).id].tolist() for name in ('if_tip','mf_tip','rf_tip','th_tip')},
                  thumb_surface_local_mm=(thumb_local*1000).tolist(),
                  all16_joints_and_cube_translation_optimized=True,
                  cube_rotation_fixed_in_proposal_only=True,proposal_translation_bounds_mm=[-20,20],
                  native_joint_limits_not_expanded=True,taught_reference_only=True)
    save_json(folder/'inputs.json',metadata)
    proposal=restore(m,z);results=[];started=time.perf_counter()
    seeds=[q0,np.clip(taught,qlo,qhi)]
    for index in range(min(a.fits,len(targets)*len(seeds))):
        local_target=targets[index//2];seed=seeds[index%2]
        ref_target=C0+B@local_target
        world=vertices[target_geom]@d.geom_xmat[target_geom].reshape(3,3).T+d.geom_xpos[target_geom]
        local_point=vertices[target_geom][np.argmin(np.linalg.norm(world-ref_target,axis=1))]
        def measure(x):
            proposal.qpos[:]=z['qpos'];proposal.qpos[:16]=x[:16];C=C0+x[16:]/1000
            proposal.qpos[-7:-4]=C;proposal.qvel[:]=0;proposal.ctrl[:]=x[:16]+lag
            mujoco.mj_forward(m,proposal)
            tasks=[]
            for gid,point,local in anchors+[(target_geom,local_point,local_target)]:
                tasks.extend((proposal.geom_xpos[gid]+proposal.geom_xmat[gid].reshape(3,3)@point-C-B@local)*1000)
            support,own=guard.contacts(proposal)
            pen=max((max(0.,-float(c.dist)*1000) for c in proposal.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return np.asarray(tasks),support,own,pen
        def residual(x):
            tasks,support,own,pen=measure(x)
            return np.r_[tasks*3,(x[:16]-q0)*.04,x[16:]*.015,
                         max(own.values(),default=0.)*1000,max(support.values(),default=0.)*1000,max(0.,pen-.8)*500]
        start=np.r_[np.clip(seed,qlo+1e-8,qhi-1e-8),[0.,0.,0.]]
        fit=least_squares(residual,start,bounds=(lower,upper),max_nfev=180,ftol=1e-8,xtol=1e-8,gtol=1e-8)
        tasks,support,own,pen=measure(fit.x);names=set();contact_rows=[]
        for c in proposal.contact:
            x,y=int(c.geom1),int(c.geom2)
            if cube not in (x,y):continue
            other=y if x==cube else x;name=m.geom(other).name
            if int(m.geom_bodyid[other]) in guard.hand and c.dist<.0004:
                names.add(name);contact_rows.append(dict(geom=name,distance_mm=float(c.dist)*1000))
        needed=set(required)|{a.finger+'_tip'};goal=motor(proposal.ctrl)
        valid=bool(needed<=names and max(own.values(),default=0.)<=1e-6 and
                   max(support.values(),default=0.)<=.15 and pen<=1 and
                   np.all(goal>=guard.lo) and np.all(goal<=guard.hi))
        row=dict(index=index,target_local_mm=(local_target*1000).tolist(),seed=index%2,
                 point_error_mm=float(np.max(np.abs(tasks))),cube_offset_mm=fit.x[16:].tolist(),
                 self_pairs_mm=own,support_pairs_mm=support,cube_penetration_mm=pen,
                 contacts=contact_rows,static_candidate=valid,goal_ticks=goal.tolist(),
                 upper_gap_mm=min(v for _,v in gaps(m,proposal,proposal.geom_xpos[cube],B,cache,True)),
                 nfev=int(fit.nfev))
        results.append(row)
        if valid:
            np.savez_compressed(folder/f'proposal{index}.npz',qpos=proposal.qpos,ctrl=proposal.ctrl)
        print(json.dumps({k:row[k] for k in ('index','static_candidate','point_error_mm','cube_offset_mm','cube_penetration_mm')}
                         |dict(self_mm=max(own.values(),default=0.),contacts=sorted(names))),flush=True)
    dynamics=[]
    for row in sorted([r for r in results if r['static_candidate']],key=lambda r:r['point_error_mm'])[:4]:
        data=restore(m,z);checker=Guard(m);initial_goal=motor(data.ctrl);target=np.array(row['goal_ticks'])
        count=max(1,int(np.ceil(np.max(np.abs(target-initial_goal))/2)))
        goals=[np.rint(initial_goal+(target-initial_goal)*(i+1)/count).astype(int) for i in range(count)]
        goals.extend(target.copy() for _ in range(134));stop=None;third=0;held=0
        for k,goal in enumerate(goals):
            data.ctrl[:]=ticks_to_sim(goal)
            for _ in range(75):
                mujoco.mj_step(m,data);stop=checker.check(data)
                if np.linalg.norm(data.qpos[-7:-4]-z['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if k>=len(goals)-54:
                    force={}
                    for ci,c in enumerate(data.contact):
                        x,y=int(c.geom1),int(c.geom2)
                        if cube not in (x,y):continue
                        other=y if x==cube else x;name=m.geom(other).name
                        if name in needed:
                            f=np.zeros(6);mujoco.mj_contactForce(m,data,ci,f);force[name]=force.get(name,0.)+float(f[0])
                    held+=1;third+=all(force.get(n,0.)>.15 for n in needed)
                if stop:break
            if stop:break
        trial=folder/f'dynamic{row["index"]}';trial.mkdir(exist_ok=True)
        np.savez_compressed(trial/'research_goals.npz',start=initial_goal,goals=goals[:k+1])
        np.savez_compressed(trial/'end_state.npz',**{key:getattr(data,key) for key in BOUND},time=data.time,
                            up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
        dynamic=dict(candidate=row['index'],goals_completed=k+1,stop=stop,worst=checker.worst,
                     end=inspect(m,data,z['up'],z['initial_cube_rotation']),
                     hold_1ms_samples=held,required_contact_fraction=third/max(1,held),
                     pose_drift_deg=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),B),
                     third_support_reached_and_held=not stop and held==4050 and third/4050>.8)
        dynamics.append(dynamic)
        save_json(trial/'review.json',dynamic)
        print(json.dumps(dict(dynamic_candidate=row['index'],stop=stop,goals_completed=k+1,
                              third_support=dynamic['third_support_reached_and_held'])),flush=True)
    report=dict(status='cooperative_contact_mode_search_not_whole_cycle_success',finger=a.finger,mode=a.mode,
                inputs=metadata,static_fits=len(results),static_candidates=sum(r['static_candidate'] for r in results),
                proposals=results,dynamic_trials=dynamics,
                full_initial_placement_replayed=False,second90_rotation_passed=False,
                devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-started)
    save_json(folder/'review.json',report)
    print(json.dumps({k:report[k] for k in ('status','static_fits','static_candidates','elapsed_seconds')}),flush=True)


if __name__=='__main__':main()
