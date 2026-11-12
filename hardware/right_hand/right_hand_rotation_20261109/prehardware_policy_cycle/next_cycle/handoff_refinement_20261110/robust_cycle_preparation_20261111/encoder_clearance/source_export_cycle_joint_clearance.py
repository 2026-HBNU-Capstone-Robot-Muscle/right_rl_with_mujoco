"""Export and verify encoder-only FK against native MuJoCo geometry."""
from pathlib import Path
import json
import numpy as np
from native_cycle import OUT,assets,restore,Guard,save_json,sha
from planning_pair_gap import PlanningPairGap
from cycle_joint_clearance import FingerClearance


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';root=base/'robust_cycle_preparation_20261111';folder=root/'encoder_clearance'
    if folder.exists():raise RuntimeError('Preserve encoder-only geometry export')
    folder.mkdir();m,initial,_,_,provenance=assets('active_ccd50');pair=[m.geom(n).id for n in ('mf_bs_collision_1','mf_tip')];geometry=PlanningPairGap(m,pair);needed=set()
    for geom in pair:
        body=int(m.geom_bodyid[geom])
        while body:needed.add(body);body=int(m.body_parentid[body])
    fields={name:getattr(m,name) for name in ('body_parentid','body_pos','body_quat','body_jntadr','body_jntnum','jnt_type','jnt_pos','jnt_axis','jnt_qposadr','qpos0')}
    fields.update(body_order=sorted(needed),geom_bodyid=m.geom_bodyid[pair],geom_pos=m.geom_pos[pair],geom_quat=m.geom_quat[pair])
    for i,geom in enumerate(pair):fields.update({f'vertices{i}':geometry.cache[geom][0],f'normals{i}':geometry.cache[geom][1]})
    target=folder/'kinematics.npz';np.savez_compressed(target,**fields);fk=FingerClearance(target);worst_position=worst_rotation=worst_gap=0.;checks=0
    trajectory=np.load(base/'initial_second_turn_hold_replay/trajectory.npz',allow_pickle=False)['qpos'];rng=np.random.default_rng(1442)
    cases=[q.copy() for q in trajectory[np.linspace(0,len(trajectory)-1,100).astype(int)]]
    for q in cases[:20]:
        for _ in range(2):extra=q.copy();extra[:16]+=rng.uniform(-.001,.001,16);cases.append(extra)
    for q in cases:
        z=dict(initial,qpos=q);d=restore(m,z);poses=fk.poses(q[:16])
        for geom,(R,t) in zip(pair,poses):worst_position=max(worst_position,float(np.max(abs(t-d.geom_xpos[geom]))));worst_rotation=max(worst_rotation,float(np.max(abs(R-d.geom_xmat[geom].reshape(3,3)))))
        worst_gap=max(worst_gap,abs(fk.gap_mm(q[:16])-geometry.gap(d,*pair)*1000));checks+=1
    assert worst_position<1e-12 and worst_rotation<1e-12 and worst_gap<1e-8,(worst_position,worst_rotation,worst_gap)
    (folder/'source_export_cycle_joint_clearance.py').write_bytes(Path(__file__).read_bytes());(folder/'source_cycle_joint_clearance.py').write_bytes((OUT/'cycle_joint_clearance.py').read_bytes())
    save_json(folder/'review.json',dict(status='encoder_only_FK_native_parity_passed',kinematics=target.relative_to(OUT).as_posix(),kinematics_sha256=sha(target),runtime_sha256=sha(OUT/'cycle_joint_clearance.py'),model_provenance=provenance,pair_names=['mf_bs_collision_1','mf_tip'],fixture_states=checks,maximum_position_difference_m=worst_position,maximum_rotation_matrix_difference=worst_rotation,maximum_gap_difference_mm=worst_gap,only_encoder_angles_required_for_static_pair_geometry=True,no_model_contact_force_used_by_runtime=True,positive_axis_gap_is_static_pair_separation_not_dynamic_certificate=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False));print(json.dumps(dict(fixtures=checks,position_m=worst_position,rotation=worst_rotation,gap_mm=worst_gap)),flush=True)


if __name__=='__main__':main()
