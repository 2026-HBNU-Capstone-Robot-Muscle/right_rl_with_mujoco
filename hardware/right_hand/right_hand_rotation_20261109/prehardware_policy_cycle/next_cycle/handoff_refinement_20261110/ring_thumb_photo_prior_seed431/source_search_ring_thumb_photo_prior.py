"""Photo-inspired RF backstop + adjustable thumb geometry, not photo registration.

The stored ring values remain intact. A separate native-admissible reference
reserves closing room inside the old bounds; no hardware limits are changed.
Cube placement is searched explicitly instead of inheriting an unrelated pose.
"""
from pathlib import Path
import argparse
import json
import time
import numpy as np
import mujoco
from scipy.optimize import differential_evolution
from scipy.spatial import ConvexHull
from native_cycle import OUT,ROOT,assets,restore,Guard,BOUND,SIM_TO_MOTOR_IDS,TICK,motor,save_json,sha
from cube_task_frame_20261105 import upright_yaw_target
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    p=argparse.ArgumentParser();p.add_argument('--seed',type=int,default=431);p.add_argument('--iterations',type=int,default=260);args=p.parse_args()
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/f'ring_thumb_photo_prior_seed{args.seed}'
    if folder.exists():raise RuntimeError('Preserve previous geometry hypotheses')
    model,initial,_,_,provenance=assets('active_ccd50');guard=Guard(model);cube=model.geom('cube').id
    source=base/'initial_to_reached_workstation90/end_state.npz';z=dict(np.load(source,allow_pickle=False));data=restore(model,z)
    record_path=ROOT/'ring_reference_readonly_20261109_124719_945552/result.json';record=json.loads(record_path.read_text());values={r['id']:r['position_ticks'] for r in record['snapshots'][0]['motors']}
    rf=np.arange(8,12);th=np.arange(12,16);sid=np.asarray(SIM_TO_MOTOR_IDS)-1
    original=np.array([(values[int(i)]-2048)*TICK for i in np.array(SIM_TO_MOTOR_IDS)[rf]])
    low=np.maximum(guard.qlo+2*TICK,(np.ceil(guard.lo[sid])+2-2048)*TICK)
    high=np.minimum(guard.qhi-2*TICK,(np.floor(guard.hi[sid])-2-2048)*TICK)
    # This new reference is not the exact photographed or recorded posture.
    ring=np.clip(original,low[rf]+np.radians(2),high[rf]-np.radians(2))
    reference=z['qpos'].copy();reference[rf]=ring
    B=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90.);quat=np.empty(4);mujoco.mju_mat2Quat(quat,B.ravel());reference[-4:]=quat
    geomdata=precompute(model,guard.hand);verts={g:v for g,v,_ in geomdata};activegeom=[g for g in geomdata if (model.body(int(model.geom_bodyid[g[0]])).name or '').startswith(('rf_','th_'))]
    tips={name:model.geom(name+'_tip').id for name in ('rf','th')};hulls={name:ConvexHull(verts[gid]) for name,gid in tips.items()}
    probe=restore(model,z);probe.qpos[:]=reference;probe.ctrl[:]=reference[:16];mujoco.mj_forward(model,probe)
    support,own=guard.contacts(probe)
    if max(own.values(),default=0)>1e-6:raise RuntimeError('Fixed ring/non-thumb combination has self collision: '+str(own))
    center0=data.geom_xpos[cube].copy();folder.mkdir();(folder/'source_search_ring_thumb_photo_prior.py').write_bytes(Path(__file__).read_bytes());rows=[];began=time.perf_counter()
    for axis,rf_sign in ((0,1),(0,-1),(2,1),(2,-1)):
        across=2 if axis==0 else 0;name=f'axis{axis}_ring{rf_sign:+d}';trial=folder/name;trial.mkdir();proposal=restore(model,z)
        def measure(x,screen=False):
            proposal.qpos[:]=reference;proposal.qpos[th]=x[:4];proposal.qpos[-7:-4]=center0+x[4:7]/1000;proposal.ctrl[:]=proposal.qpos[:16]
            mujoco.mj_forward(model,proposal);C=proposal.geom_xpos[cube];fits=[]
            for finger,sign in (('rf',rf_sign),('th',-rf_sign)):
                point=np.zeros(3);point[axis]=sign*27.45;point[1]=x[7];point[across]=x[8]
                tip=tips[finger];h=hulls[finger];R=proposal.geom_xmat[tip].reshape(3,3);local=R.T@(C+B@point/1000-proposal.geom_xpos[tip]);planes=h.equations[:,:3]@local+h.equations[:,3];j=int(np.argmax(planes))
                fits.append((float(planes[j]*1000),float(np.linalg.norm(R@h.equations[j,:3]+sign*B[:,axis]))))
            support,own=guard.contacts(proposal);pen=max((-float(c.dist)*1000 for c in proposal.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            outside=max(0.,float(np.linalg.norm(C-initial['initial_cube_center'])*1000)-79.)
            screen_value=min((v for _,v in gaps(model,proposal,C,B,activegeom,True)),default=100.) if screen else None
            return fits,max(own.values(),default=0.),max(support.values(),default=0.),pen,outside,screen_value
        def objective(x):
            fits,selfpen,supportpen,cubepen,outside,_=measure(x)
            return sum(abs(gap+.03)+normal*2.5 for gap,normal in fits)+selfpen*2000+supportpen*1200+max(0.,cubepen-.75)*800+outside*100+np.linalg.norm(x[4:7])*.005
        limits=list(zip(low[th],high[th]))+[(-60,60)]*3+[(0.,23.),(-22.,22.)]
        rng=np.random.default_rng(args.seed+axis*10+rf_sign);population=rng.uniform(np.array(limits)[:,0],np.array(limits)[:,1],(100,9))
        # Near-contact seeds search cube centers near the fixed ring tip.
        for i in range(15):
            point=np.zeros(3);point[axis]=rf_sign*39.;point[1]=rng.uniform(0,20);point[across]=rng.uniform(-15,15)
            population[i,:4]=np.clip(data.qpos[th]+rng.normal(0,.15,4),low[th],high[th])
            population[i,4:7]=np.clip((probe.geom_xpos[tips['rf']]-B@point/1000-center0)*1000,-60,60)
        fit=differential_evolution(objective,limits,seed=args.seed+axis*10+rf_sign,init=population,maxiter=args.iterations,tol=1e-7,polish=True,workers=1)
        pair,selfpen,supportpen,cubepen,outside,screen=measure(fit.x,True);C=proposal.geom_xpos[cube].copy();contacts=[];present=[]
        for finger,sign in (('rf',rf_sign),('th',-rf_sign)):
            normal=sign*B[:,axis];items=[]
            for c in proposal.contact:
                if {int(c.geom1),int(c.geom2)}!={cube,tips[finger]} or c.dist>=.0002:continue
                n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);local=(c.pos-C)@B*1000
                items.append(dict(point_cube_frame_mm=local.tolist(),normal_alignment=float(n@normal),distance_mm=float(c.dist*1000)))
            present.append(any(item['normal_alignment']>.8 and item['point_cube_frame_mm'][1]>-8. for item in items))
            target=np.zeros(3);target[axis]=sign*27.45;target[1]=fit.x[7];target[across]=fit.x[8]
            contacts.append(dict(finger=finger,target_cube_frame_mm=target.tolist(),contacts=items))
        cert=SweepCertificate(model,guard.hand,.3);lower,bad=cert.query(proposal,C,B);goal=motor(proposal.ctrl)
        contact_valid=bool(all(present) and selfpen<=1e-6 and supportpen<=.15 and cubepen<=1 and outside==0 and np.all(goal>=guard.lo) and np.all(goal<=guard.hi))
        clear_valid=bool(contact_valid and not bad and lower>.3)
        np.savez_compressed(trial/'proposal.npz',**{key:getattr(proposal,key) for key in BOUND},time=proposal.time,up=initial['up'],initial_cube_rotation=initial['initial_cube_rotation'],initial_cube_center=initial['initial_cube_center'])
        row=dict(profile=name,axis=axis,ring_sign=rf_sign,objective=float(fit.fun),nfev=int(fit.nfev),center=C.tolist(),shift_from_reached_station_mm=fit.x[4:7].tolist(),contacts=contacts,
            point_plane_gaps_and_normal_errors=pair,self_mm=selfpen,support_mm=supportpen,cube_penetration_mm=cubepen,RF_TH_upper_screen_mm=screen,upper_certificate_lower_mm=lower,
            bad_upper_geoms=[model.geom(int(cert.ids[i])).name for i in bad],static_opposed_contact_candidate=contact_valid,static_clear_opposed_grip_candidate=clear_valid,
            proposal=(trial/'proposal.npz').relative_to(OUT).as_posix(),proposal_sha256=sha(trial/'proposal.npz'))
        rows.append(row);save_json(trial/'review.json',row);save_json(folder/'progress.json',dict(rows=rows))
        print(json.dumps({k:row[k] for k in ('profile','objective','shift_from_reached_station_mm','static_opposed_contact_candidate','static_clear_opposed_grip_candidate','upper_certificate_lower_mm')}),flush=True)
    save_json(folder/'review.json',dict(status='photo_inspired_fixed_ring_backstop_adjustable_thumb_geometry',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),
        recorded_ring_source_sha256=sha(record_path),recorded_ring_radians=original.tolist(),native_admissible_hypothesis_ring_radians=ring.tolist(),ring_difference_from_record_deg=np.degrees(ring-original).tolist(),
        model_provenance=provenance,rows=rows,photo_used_as_qualitative_arrangement_not_camera_registration=True,exact_recorded_values_preserved_but_not_used_as_native_goal=True,
        cube_fixed_only_in_hypothetical_geometry_copies=True,arrival_path_passed=False,free_object_hold_passed=False,finite_search_not_infeasibility_proof=True,
        old_limits_and_collision_geometry_not_changed=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began))


if __name__=='__main__':main()
