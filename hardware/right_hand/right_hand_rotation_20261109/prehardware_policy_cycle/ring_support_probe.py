"""Bounded RF tip/link contact search at a saved lower grip; no new limits."""
import time
import json
import numpy as np
import mujoco
from scipy.optimize import least_squares
from native_cycle import OUT,assets,restore,Guard,motor,ticks_to_sim,BOUND,save_json,TICK,SIM_TO_MOTOR_IDS
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from inspect_endpoints import inspect


def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--finger',choices=('rf','mf'),default='rf')
    p.add_argument('--transient',action='store_true');p.add_argument('--model',choices=('active','archived'),default='archived')
    p.add_argument('--source',default='baseline_archived_zero/end_state.npz');opts=p.parse_args()
    began=time.perf_counter();m,_,_,_,provenance=assets(opts.model)
    z=dict(np.load(OUT/opts.source,allow_pickle=False));d=restore(m,z);g=Guard(m)
    cube=m.geom('cube').id;C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
    cache=precompute(m,g.hand);rf=np.arange(8,12) if opts.finger=='rf' else np.arange(4,8);lag=d.ctrl-d.qpos[:16]
    sid=np.array(SIM_TO_MOTOR_IDS)-1
    lower=(np.ceil(g.lo[sid])+2-2048)*TICK-lag;upper=(np.floor(g.hi[sid])-2-2048)*TICK-lag
    rf_geoms=[]
    for gid,v,e in cache:
        body=m.body(int(m.geom_bodyid[gid])).name or ''
        eligible=(m.geom_contype[gid]&m.geom_conaffinity[cube]) or (m.geom_contype[cube]&m.geom_conaffinity[gid])
        if body.startswith(opts.finger+'_') and eligible and m.geom(gid).name in (opts.finger+'_tip',opts.finger+'_ds_collision_1',opts.finger+'_md_collision_1'):
            rf_geoms.append((gid,v))
    targets=[]
    for fixed_axis in (0,2):
        for sign in (-1,1):
            for y in (-3,12,24):
                for across in (-18,0,18):
                    p=np.zeros(3);p[fixed_axis]=sign*(27.2 if opts.transient else 27.6);p[1]=y;p[2 if fixed_axis==0 else 0]=across
                    targets.append(C+B@p/1000)
    for x in (-18,0,18):
        for zz in (-18,0,18):targets.append(C+B@np.array([x,27.6,zz])/1000)
    taught=np.array([3053,2829,2961,3018])*TICK-2048*TICK
    if opts.finger=='mf':
        from native_cycle import ROOT
        actual=json.loads((ROOT/'middle_clearance_probe_20261106_230902_998326/result.json').read_text())
        taught=(np.asarray(actual['middle_probe_position_ticks'])[sid[rf]]-2048)*TICK
    starts=(d.qpos[rf].copy(),np.clip(taught,lower[rf],upper[rf]))
    candidates=[];nearest=[]
    for gid,vertices in rf_geoms:
        world=vertices@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid]
        for target_index,target in enumerate(targets):
            local=vertices[int(np.argmin(np.linalg.norm(world-target,axis=1)))]
            for seed_index,seed in enumerate(starts):
                proposal=restore(m,z)
                def residual(x):
                    proposal.qpos[:]=z['qpos'];proposal.qpos[rf]=x;proposal.ctrl[:]=proposal.qpos[:16]+lag
                    mujoco.mj_forward(m,proposal);G=proposal.geom_xmat[gid].reshape(3,3)
                    error=(proposal.geom_xpos[gid]+G@local-target)*1000
                    support,own=g.contacts(proposal);pen=0.
                    for c in proposal.contact:
                        if cube in (int(c.geom1),int(c.geom2)):pen=max(pen,-float(c.dist)*1000)
                    return np.r_[error,(x-seed)*.1,max(own.values(),default=0.)*500,
                                 max(support.values(),default=0.)*500,max(0.,pen-.8)*100]
                fit=least_squares(residual,np.clip(seed,lower[rf],upper[rf]),bounds=(lower[rf],upper[rf]),
                                  max_nfev=50,ftol=1e-7,xtol=1e-7,gtol=1e-7)
                error=float(np.linalg.norm(residual(fit.x)[:3]));support,own=g.contacts(proposal)
                contacts=[]
                for c in proposal.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    if cube not in (a,b):continue
                    other=b if a==cube else a
                    if (m.body(int(m.geom_bodyid[other])).name or '').startswith(opts.finger+'_') and c.dist<.0004:
                        height=float(-(c.pos-C)@B[:,1]*1000)
                        if height<55/6:contacts.append({'geom':m.geom(other).name,'height_mm':height,'distance_mm':float(c.dist)*1000})
                goal=motor(proposal.qpos[:16]+lag)
                valid=(not own and max(support.values(),default=0.)<=.15 and contacts and
                       np.all(goal>=g.lo) and np.all(goal<=g.hi))
                gap=min(value for _,value in gaps(m,proposal,C,B,cache,True)) if valid else None
                valid=bool(valid and (opts.transient or gap>.3))
                row={'geom':m.geom(gid).name,'target':target_index,'seed':seed_index,'point_error_mm':error,
                     'RF_model_contacts':contacts,'self_pairs_mm':own,'upper_gap_mm':gap,
                     'native_goal_ticks':goal.tolist(),'static_candidate':valid}
                nearest.append(row)
                if valid:candidates.append(row)
        print(json.dumps({'geom':m.geom(gid).name,'fits':len(nearest),'static_candidates':len(candidates)}),flush=True)
    label='ring_tip_and_link_support' if opts.finger=='rf' and not opts.transient else opts.finger+'_transient_support'
    folder=OUT/label;folder.mkdir(exist_ok=True)
    dynamics=[]
    for index,row in enumerate(sorted(candidates,key=lambda r:r['point_error_mm'])[:4]):
        d=restore(m,z);guard=Guard(m);start=motor(d.ctrl);target=np.array(row['native_goal_ticks'])
        n=max(1,int(np.ceil(np.max(abs(target-start))/2)));rows=[np.rint(start+(target-start)*k/n).astype(int) for k in range(1,n+1)]
        rows.extend(target.copy() for _ in range(134));stop=None
        hold_n=three_n=0
        for k,goal in enumerate(rows):
            d.ctrl[:]=ticks_to_sim(goal)
            for _ in range(75):
                mujoco.mj_step(m,d);stop=guard.check(d)
                if k>=len(rows)-54:
                    force={};Bnow=d.geom_xmat[cube].reshape(3,3);Cnow=d.geom_xpos[cube]
                    for ci,c in enumerate(d.contact):
                        aa,bb=int(c.geom1),int(c.geom2)
                        if cube not in (aa,bb):continue
                        other=bb if aa==cube else aa;name=m.geom(other).name
                        if name in ('if_tip','th_tip',opts.finger+'_tip') and -(c.pos-Cnow)@Bnow[:,1]*1000<55/6:
                            f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);force[name]=force.get(name,0.)+float(f[0])
                    hold_n+=1;three_n+=all(force.get(name,0.)>.2 for name in ('if_tip','th_tip',opts.finger+'_tip'))
                if stop:break
            if stop:break
        end=inspect(m,d,z['up'],z['initial_cube_rotation'])
        dynamics.append({'source_candidate':row,'completed_goals':k+1,'stop':stop,'worst':guard.worst,'endpoint':end,
                         'three_lower_fingertips_fraction':three_n/max(1,hold_n),'hold_samples':hold_n})
        np.savez_compressed(folder/f'end{index}.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,
                            up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    result={'status':'offline_ring_tip_and_link_support_search','model_provenance':provenance,
            'finger':opts.finger,'transient_support_not_final_upper_clearance_pose':opts.transient,'source':opts.source,
            'taught_reference_only_native_ranges_not_expanded':True,
            'candidate_cube_pose_and_other_fingers_fixed_only_during_IK':True,
            'dynamic_cube_free_not_reset_during_each_suffix':True,
            'fits':len(nearest),'static_candidates':len(candidates),
            'nearest':sorted(nearest,key=lambda r:r['point_error_mm'])[:12],
            'dynamic_probes':dynamics,'task_success_certified':False,'devices_opened':0,
            'hardware_execution_allowed':False,'elapsed_seconds':time.perf_counter()-began}
    save_json(folder/'review.json',result)
    print(json.dumps({key:result[key] for key in ('status','fits','static_candidates','elapsed_seconds')}),flush=True)


if __name__=='__main__':main()
