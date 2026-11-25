"""Bounded global RF geometry search; no motion or hardware-limit changes.

Compare the original model bound with the single recorded ID14 angle as a
geometry hypothesis. The latter is never an executable/qualified motion.
"""
from pathlib import Path
import json
import time
import argparse
import numpy as np
import mujoco
from scipy.optimize import differential_evolution
from native_cycle import OUT,ROOT,assets,restore,Guard,BOUND,TICK,SIM_TO_MOTOR_IDS,motor,save_json,sha
from convex_pair_gap import ConvexPairGap


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--transient',action='store_true')
    parser.add_argument('--original-bound-only',action='store_true');args=parser.parse_args()
    folder=OUT/('next_cycle/handoff_refinement_20261110/ring_global_reach'+('_transient' if args.transient else ''))
    if folder.exists():raise RuntimeError('Preserve previous global reach study')
    model,initial,_,_,provenance=assets('active_ccd50');guard=Guard(model);cube=model.geom('cube').id
    pool_path=OUT/'next_cycle/handoff_refinement_20261110/early_CCD50_states/state_pool.npz'
    pool=np.load(pool_path,allow_pickle=False);index=int(np.flatnonzero(pool['indices']==1700)[0]);z=dict(initial)
    z.update({key:pool[key][index].copy() for key in BOUND});z['time']=float(pool['time'][index])
    d=restore(model,z);rf=np.arange(8,12);sid=np.array(SIM_TO_MOTOR_IDS)-1;lag=d.ctrl-d.qpos[:16]
    geoms=[model.geom(name).id for name in ('rf_tip','rf_ds_collision_1','rf_md_collision_1')]
    distance=ConvexPairGap(model,[cube]+geoms);C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
    low=np.maximum(guard.qlo,(np.ceil(guard.lo[sid])+2-2048)*TICK-lag)
    high=np.minimum(guard.qhi,(np.floor(guard.hi[sid])-2-2048)*TICK-lag)
    source=ROOT/'ring_reference_readonly_20261109_124719_945552/result.json'
    measured=json.loads(source.read_text());actual={row['id']:row['position_ticks'] for row in measured['snapshots'][0]['motors']}
    folder.mkdir();rows=[];began=time.perf_counter()
    profiles=('original_model_bound',) if args.original_bound_only else ('original_model_bound','recorded_ID14_geometry_hypothesis')
    for profile in profiles:
        upper=high.copy()
        if profile!='original_model_bound':upper[9]=(actual[14]-2048)*TICK
        for geom in geoms:
            proposal=restore(model,z)
            def measure(x):
                proposal.qpos[:]=z['qpos'];proposal.qpos[rf]=x;proposal.ctrl[:]=proposal.qpos[:16]+lag
                mujoco.mj_forward(model,proposal)
                separation=distance.gap(proposal,cube,geom)*1000
                support,own=guard.contacts(proposal)
                height=float(-(proposal.geom_xpos[geom]-C)@B[:,1]*1000)
                pen=max((-float(c.dist)*1000 for c in proposal.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
                return separation,own,max(support.values(),default=0.),height,pen
            def objective(x):
                gap,own,support,height,pen=measure(x)
                return (abs(gap+.05)+max(own.values(),default=0.)*1000+support*1000+
                        (0. if args.transient else 2*max(0.,height-5))+500*max(0.,pen-.8))
            fit=differential_evolution(objective,list(zip(low[rf],upper[rf])),seed=1700+geom,
                popsize=12,maxiter=80,tol=.001,polish=True,workers=1)
            gap,own,support,height,pen=measure(fit.x);contacts=[]
            for contact in proposal.contact:
                first,second=int(contact.geom1),int(contact.geom2)
                if cube not in (first,second):continue
                other=second if first==cube else first
                if (model.geom(other).name or '').startswith('rf_') and contact.dist<.0001:
                    contacts.append(dict(geom=model.geom(other).name,distance_mm=float(contact.dist*1000),
                                         height_mm=float(-(contact.pos-C)@B[:,1]*1000)))
            original_range_violation=max(0.,float(np.max(guard.qlo-proposal.qpos[:16])),float(np.max(proposal.qpos[:16]-guard.qhi)))
            candidate=bool(contacts and (args.transient or any(row['height_mm']<55/6 for row in contacts)) and
                           max(own.values(),default=0.)<=1e-6 and support<=.15 and pen<=1.)
            row=dict(profile=profile,geom=model.geom(geom).name,objective=float(fit.fun),nfev=int(fit.nfev),
                selected_signed_convex_gap_mm=gap,self_pairs_mm=own,cube_penetration_mm=pen,
                RF_contacts=contacts,original_range_violation_rad=original_range_violation,
                static_contact_geometry_candidate=candidate,not_a_motion_or_holding_pass=True,
                hypothetical_ring_ticks=(proposal.qpos[rf]/TICK+2048).tolist(),
                hypothetical_command_motor_ticks=motor(proposal.ctrl).tolist())
            rows.append(row)
            np.savez_compressed(folder/(profile+'_'+model.geom(geom).name+'.npz'),qpos=proposal.qpos,ctrl=proposal.ctrl)
            print(json.dumps({key:row[key] for key in ('profile','geom','selected_signed_convex_gap_mm','static_contact_geometry_candidate','original_range_violation_rad')}),flush=True)
    report=dict(status='bounded_global_RF_reach_geometry_comparison',model_provenance=provenance,
        rows=rows,source_pool_sha256=sha(pool_path),recorded_RF_source_sha256=sha(source),
        hypothetical_ID14_upper_ticks=actual[14],other_joint_bounds_unchanged=True,
        model_and_physical_joint_limits_not_modified=True,
        cube_and_other_fingers_fixed_in_geometry_proposals_only=True,
        temporary_support_may_occupy_upper_sweep=args.transient,final_work_grip_not_certified=True,
        optimizer_budget_does_not_prove_general_infeasibility=True,
        actual_motion_evaluations=0,devices_opened=0,hardware_execution_allowed=False,
        elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',report)
    (folder/'source_search_ring_reach_global.py').write_bytes(Path(__file__).read_bytes())


if __name__=='__main__':main()
