"""Global four-joint MF lower-contact proposals at the actual rotated cube."""
from pathlib import Path
import argparse
import json
import time
import numpy as np
import mujoco
from scipy.optimize import differential_evolution
from native_cycle import OUT,assets,restore,Guard,motor,TICK,SIM_TO_MOTOR_IDS,save_json,sha
from convex_pair_gap import ConvexPairGap


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--source-state');parser.add_argument('--tag',default='actual_cube_MF_global_contact');args=parser.parse_args()
    if not args.tag.replace('_','').isalnum():raise ValueError('Local tag required')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/args.tag
    if folder.exists():raise RuntimeError('Preserve previous global MF study')
    source=base/'first90_RF_shape_before_adduction/RF_park_MF_then_RF_shape_then_adduct/phase0_end_state.npz'
    if args.source_state:
        source=(OUT/args.source_state).resolve()
        if not source.is_relative_to(OUT.resolve()):raise ValueError('Local research source required')
    model,_,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id;tip=model.geom('mf_tip').id;guard=Guard(model)
    z=dict(np.load(source,allow_pickle=False));data=restore(model,z);C=data.geom_xpos[cube].copy();B=data.geom_xmat[cube].reshape(3,3).copy()
    active=np.arange(4,8);sid=np.array(SIM_TO_MOTOR_IDS)-1;lag=data.ctrl-data.qpos[:16]
    low=np.maximum(guard.qlo,(np.ceil(guard.lo[sid])+2-2048)*TICK-lag);high=np.minimum(guard.qhi,(np.floor(guard.hi[sid])-2-2048)*TICK-lag)
    pair=ConvexPairGap(model,[cube,tip]);folder.mkdir();(folder/'source_search_MF_actual_cube_global.py').write_bytes(Path(__file__).read_bytes());rows=[];began=time.perf_counter()
    for seed in (31,71,131):
        proposal=restore(model,z)
        def measure(x):
            proposal.qpos[:]=z['qpos'];proposal.qpos[active]=x;proposal.ctrl[:]=proposal.qpos[:16]+lag;mujoco.mj_forward(model,proposal)
            support,own=guard.contacts(proposal);gap=pair.gap(proposal,cube,tip)*1000
            center=(proposal.geom_xpos[tip]-C)@B*1000
            penetration=max((-float(c.dist)*1000 for c in proposal.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return gap,center,support,own,penetration
        def objective(x):
            gap,center,support,own,penetration=measure(x)
            return abs(gap+.025)+2*max(0.,-center[1])+1500*max(own.values(),default=0.)+1000*max(support.values(),default=0.)+500*max(0.,penetration-.8)
        random=np.random.default_rng(seed);population=random.uniform(low[active],high[active],(48,4));population[0]=np.clip(data.qpos[active],low[active],high[active])
        for i,posture in enumerate(([35,-30,60,60],[70,-30,60,90],[20,-45,90,90]),1):population[i]=np.clip(np.radians(posture),low[active],high[active])
        fit=differential_evolution(objective,list(zip(low[active],high[active])),seed=seed,init=population,maxiter=140,tol=1e-6,polish=True,workers=1)
        gap,center,support,own,penetration=measure(fit.x);contacts=[]
        for contact in proposal.contact:
            if {int(contact.geom1),int(contact.geom2)}=={cube,tip} and contact.dist<.0002:
                normal=contact.frame[:3]*(1 if int(contact.geom1)==cube else -1)
                contacts.append(dict(distance_mm=float(contact.dist*1000),cube_frame_mm=((contact.pos-C)@B*1000).tolist(),cube_frame_normal=(normal@B).tolist()))
        goal=motor(proposal.ctrl);candidate=bool(contacts and any(c['cube_frame_mm'][1]>-5 for c in contacts) and max(own.values(),default=0.)<=1e-6 and max(support.values(),default=0.)<=.15 and penetration<=1 and np.all(goal>=guard.lo) and np.all(goal<=guard.hi))
        proposal_path=folder/f'seed{seed}_proposal.npz';np.savez_compressed(proposal_path,qpos=proposal.qpos,ctrl=proposal.ctrl)
        row=dict(finger='mf',seed=seed,nfev=int(fit.nfev),objective=float(fit.fun),tip_cube_gap_axis_lower_mm=gap,tip_center_cube_frame_mm=center.tolist(),contacts=contacts,
            self_pairs_mm=own,cube_penetration_mm=penetration,static_contact_candidate=candidate,proposal=proposal_path.relative_to(OUT).as_posix(),proposal_sha256=sha(proposal_path))
        rows.append(row);save_json(folder/'progress.json',dict(rows=rows));print(json.dumps({key:row[key] for key in ('seed','tip_cube_gap_axis_lower_mm','tip_center_cube_frame_mm','static_contact_candidate')}),flush=True)
    save_json(folder/'review.json',dict(status='global_MF_lower_contact_at_actual_cube',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),model_provenance=provenance,rows=rows,
        object_fixed_only_in_geometry_proposals=True,original_native_joint_bounds_and_geometry_preserved=True,finite_search_not_general_infeasibility_proof=True,
        new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began))


if __name__=='__main__':main()
