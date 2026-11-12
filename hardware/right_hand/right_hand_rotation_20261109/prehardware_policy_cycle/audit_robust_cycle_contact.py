"""Inspect the MF self-contact failure and contact-convergence sensitivity."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,save_json,sha
from convex_pair_gap import ConvexPairGap


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';root=base/'robust_cycle_preparation_20261111';folder=root/'contact_diagnosis'
    if folder.exists():raise RuntimeError('Preserve prior geometry diagnosis')
    folder.mkdir();(folder/'source_audit_robust_cycle_contact.py').write_bytes(Path(__file__).read_bytes())
    case=root/'failure_checkpoints/initial_cube_worldX_plus1mm';source=case/'before_rejected_1ms_state.npz';z=dict(np.load(source,allow_pickle=False));rows=[]
    for iterations in (50,100,200):
        m,_,_,_,provenance=assets('active_ccd50');m.opt.ccd_iterations=iterations;d=restore(m,z);g=Guard(m);a=m.geom('mf_bs_collision_1').id;b=m.geom('mf_tip').id;convex=ConvexPairGap(m,[a,b]);distance=np.zeros(6)
        static=dict(native_geom_distance_mm=float(mujoco.mj_geomDistance(m,d,a,b,.03,distance)*1000),independent_convex_axis_gap_mm=convex.gap(d,a,b)*1000)
        mujoco.mj_step(m,d);reasons=g.check(d)
        row=dict(CCD_iterations=iterations,static_before_rejected_step=static,replayed_1ms_guard=reasons,self_pairs_mm=g.last['self_pairs_mm'],terminal_qpos=d.qpos.tolist())
        if iterations==50:
            old=np.load(case/'end_state.npz',allow_pickle=False);row['exact_failure_qpos_match']=np.array_equal(d.qpos,old['qpos']);assert row['exact_failure_qpos_match']
        rows.append(row)
    save_json(folder/'review.json',dict(status='fixed_source_1ms_MF_self_contact_comparison',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),rows=rows,model_provenance=provenance,only_numerical_CCD_option_changed_in_RAM=True,source_model_not_changed=True,source_guard_not_changed=True,reported_small_depth_is_model_geometry_not_physical_measurement=True,independent_axis_negative_value_is_not_penetration_depth=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False));print(json.dumps(rows),flush=True)


if __name__=='__main__':main()
