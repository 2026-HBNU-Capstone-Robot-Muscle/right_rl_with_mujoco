"""Compare native MF/palm contacts with convex geometry and CCD iteration caps."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,save_json,sha
from convex_pair_gap import ConvexPairGap


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/palm_contact_convergence'
    if folder.exists():raise RuntimeError('Preserve previous convergence audit')
    m,initial,_,_,provenance=assets('active_ccd50');tip=m.geom('mf_tip').id
    palms=[index for index in range(m.ngeom) if (m.geom(index).name or '').startswith('palm_collision_')]
    gap=ConvexPairGap(m,[tip]+palms);poses=[];source_hashes={}
    pool_path=OUT/'next_cycle/handoff_refinement_20261110/early_CCD50_states/state_pool.npz'
    pool=np.load(pool_path,allow_pickle=False);pos=int(np.flatnonzero(pool['indices']==1700)[0]);state=dict(initial)
    state.update({key:pool[key][pos].copy() for key in BOUND});state['time']=float(pool['time'][pos])
    poses.append(('checkpoint1700',state,[]));source_hashes[pool_path.relative_to(OUT).as_posix()]=sha(pool_path)
    for tag in ('whole_hand_recovery_cp1700_w0.15','whole_hand_recovery_cp1700_w0.15_coupled'):
        path=OUT/'next_cycle'/tag/'end_state.npz';state=dict(np.load(path,allow_pickle=False))
        source_hashes[path.relative_to(OUT).as_posix()]=sha(path);poses.append((tag,state,[]))
        d=restore(m,state);guard=Guard(m);violations=[]
        for step in range(50):
            mujoco.mj_step(m,d);reasons=guard.check(d)
            if reasons:violations.append(dict(substep=step+1,reasons=reasons,self_pairs_mm=guard.last['self_pairs_mm']))
            if step+1 in (30,47,50):
                sample={key:getattr(d,key).copy() for key in BOUND};sample['time']=d.time
                poses.append((tag+f'_held{step+1}ms',sample,list(violations)))
            if any(reason!='self_mm' for reason in reasons) or guard.last['self_mm']>.01:break
    rows=[]
    for name,state,violations in poses:
        for iterations in (50,100,200,500):
            m.opt.ccd_iterations=iterations;d=restore(m,state);pairs=[]
            for palm in palms:
                contacts=[float(contact.dist*1000) for contact in d.contact
                          if {int(contact.geom1),int(contact.geom2)}=={tip,palm}]
                native=float(mujoco.mj_geomDistance(m,d,tip,palm,.1,np.zeros(6)))*1000
                sat=gap.gap(d,tip,palm)*1000
                pairs.append(dict(palm=m.geom(palm).name,convex_signed_axis_gap_mm=sat,
                                  native_distance_mm=native,native_contact_distances_mm=contacts))
            rows.append(dict(source=name,iterations=iterations,pairs=pairs,
                original_guard_violations_during_diagnostic_generation=violations,
                unqualified_diagnostic_pose_not_a_motion_pass=True))
        print(json.dumps(dict(source=name,nearest_convex_gap_mm=min(p['convex_signed_axis_gap_mm'] for p in pairs))),flush=True)
    m.opt.ccd_iterations=50
    contradictions=[dict(source=row['source'],iterations=row['iterations'],**pair) for row in rows for pair in row['pairs']
                    if pair['convex_signed_axis_gap_mm']>.0001 and any(x<0 for x in pair['native_contact_distances_mm'])]
    folder.mkdir();report=dict(status='native_vs_convex_MF_palm_contact_audit',model_provenance=provenance,
        source_hashes=source_hashes,geometry_source_sha256=sha(Path(__file__).with_name('convex_pair_gap.py')),
        states=len(poses),rows=rows,positive_convex_gap_with_negative_native_contact=contradictions,
        original_model_file_unchanged=True,no_hardware_contact_tolerance_inferred=True,
        original_guard_violations_preserved=True,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',report)
    (folder/'source_audit_palm_contact_convergence.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(states=len(poses),comparisons=len(rows),convex_native_contact_contradictions=len(contradictions))),flush=True)


if __name__=='__main__':main()
