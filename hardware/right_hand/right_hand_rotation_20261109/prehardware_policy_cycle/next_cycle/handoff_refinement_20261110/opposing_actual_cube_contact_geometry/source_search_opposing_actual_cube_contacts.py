"""Search matching opposite cube faces, rather than two adjacent push contacts."""
from pathlib import Path
import json
import time
import numpy as np
import mujoco
from scipy.optimize import differential_evolution
from scipy.spatial import ConvexHull
from native_cycle import OUT,assets,restore,Guard,motor,TICK,SIM_TO_MOTOR_IDS,save_json,sha
from upper_layer_cylinder_clearance_20261105 import precompute


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'opposing_actual_cube_contact_geometry'
    if folder.exists():raise RuntimeError('Preserve previous opposing contact study')
    source=base/'first90_RF_shape_before_adduction/RF_park_MF_then_RF_shape_then_adduct/phase0_end_state.npz'
    model,_,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id;guard=Guard(model);z=dict(np.load(source,allow_pickle=False));data=restore(model,z)
    C=data.geom_xpos[cube].copy();B=data.geom_xmat[cube].reshape(3,3).copy();lag=data.ctrl-data.qpos[:16];sid=np.array(SIM_TO_MOTOR_IDS)-1
    low=np.maximum(guard.qlo,(np.ceil(guard.lo[sid])+2-2048)*TICK-lag);high=np.minimum(guard.qhi,(np.floor(guard.hi[sid])-2-2048)*TICK-lag)
    geometry={g:v for g,v,_ in precompute(model,guard.hand)}
    folder.mkdir();(folder/'source_search_opposing_actual_cube_contacts.py').write_bytes(Path(__file__).read_bytes());rows=[];began=time.perf_counter()
    for profile,axis,mf_sign in (('opposed_X',0,1),('opposed_Z',2,-1)):
        across=2 if axis==0 else 0
        for finger,active,sign in (('mf',np.arange(4,8),mf_sign),('th',np.arange(12,16),-mf_sign)):
            tip=model.geom(finger+'_tip').id;vertices=geometry[tip];hull=ConvexHull(vertices);normals=hull.equations[:,:3];offsets=-hull.equations[:,3]
            local_target=np.zeros(3);local_target[axis]=sign*27.45;local_target[1]=10.;local_target[across]=15.;point=C+B@local_target/1000;cube_normal=sign*B[:,axis]
            proposal=restore(model,z)
            def measure(x):
                proposal.qpos[:]=z['qpos'];proposal.qpos[active]=x;proposal.ctrl[:]=proposal.qpos[:16]+lag;mujoco.mj_forward(model,proposal)
                R=proposal.geom_xmat[tip].reshape(3,3);local=R.T@(point-proposal.geom_xpos[tip]);planes=normals@local-offsets;j=int(np.argmax(planes))
                plane_gap=float(planes[j]*1000);normal_error=float(np.linalg.norm(R@normals[j]+cube_normal));support,own=guard.contacts(proposal)
                penetration=max((-float(c.dist)*1000 for c in proposal.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
                return plane_gap,normal_error,support,own,penetration
            def objective(x):
                gap,normal,support,own,penetration=measure(x)
                return abs(gap+.025)+normal*2+1500*max(own.values(),default=0.)+1000*max(support.values(),default=0.)+500*max(0.,penetration-.8)
            random=np.random.default_rng(813+axis+int(active[0]));population=random.uniform(low[active],high[active],(48,4));population[0]=np.clip(data.qpos[active],low[active],high[active])
            seeds=([35,-30,60,60],[70,-30,60,90],[20,-45,90,90]) if finger=='mf' else ([60,40,40,60],[20,70,50,80],[80,10,70,50])
            for index,posture in enumerate(seeds,1):population[index]=np.clip(np.radians(posture),low[active],high[active])
            fit=differential_evolution(objective,list(zip(low[active],high[active])),seed=813+axis+int(active[0]),init=population,maxiter=180,tol=1e-6,polish=True,workers=1)
            gap,normal,support,own,penetration=measure(fit.x);contacts=[]
            for contact in proposal.contact:
                if {int(contact.geom1),int(contact.geom2)}!={cube,tip} or contact.dist>=.0002:continue
                actual_normal=contact.frame[:3]*(1 if int(contact.geom1)==cube else -1)
                local=(contact.pos-C)@B*1000;alignment=float(actual_normal@cube_normal)
                contacts.append(dict(cube_local_mm=local.tolist(),normal_alignment=alignment,distance_mm=float(contact.dist*1000)))
            goal=motor(proposal.ctrl)
            candidate=bool(contacts and any(c['normal_alignment']>.8 and np.linalg.norm(np.array(c['cube_local_mm'])-local_target)<8 for c in contacts)
                and max(own.values(),default=0.)<=1e-6 and max(support.values(),default=0.)<=.15 and penetration<=1 and np.all(goal>=guard.lo) and np.all(goal<=guard.hi))
            path=folder/(profile+'_'+finger+'_proposal.npz');np.savez_compressed(path,qpos=proposal.qpos,ctrl=proposal.ctrl)
            row=dict(grasp_profile=profile,finger=finger,target_cube_frame_mm=local_target.tolist(),nfev=int(fit.nfev),objective=float(fit.fun),target_point_plane_gap_mm=gap,
                face_normal_error=normal,contacts=contacts,self_pairs_mm=own,cube_penetration_mm=penetration,static_contact_candidate=candidate,
                proposal=path.relative_to(OUT).as_posix(),proposal_sha256=sha(path))
            rows.append(row);save_json(folder/'progress.json',dict(rows=rows));print(json.dumps({key:row[key] for key in ('grasp_profile','finger','target_point_plane_gap_mm','face_normal_error','static_contact_candidate')}),flush=True)
    save_json(folder/'review.json',dict(status='opposite_face_contacts_at_actual_cube_geometry',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),model_provenance=provenance,rows=rows,
        material_contact_point_not_fixed_to_original_vertex=True,cube_fixed_only_in_proposal_copies=True,finite_search_not_general_infeasibility_proof=True,
        original_guards_and_joint_limits_preserved=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began))


if __name__=='__main__':main()
