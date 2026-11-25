"""Isolated CCD diagnostic; original model and geometry remain untouched."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,save_json,sha
from convex_distance_check_20261104 import CheckedDistance


def main():
    bit=int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD);rows=[];geometries=[]
    cases=['next_cycle/handoff_refinement_20261110/fixed_grip_search0/proposal.npz',
           'next_cycle/handoff_refinement_20261110/fixed_grip_search1/proposal.npz',
           'next_cycle/handoff_integer_load_first_v6/end_state.npz']
    for variant in ('original','legacy_MPR','no_midphase','ccd50','ccd100','ccd200'):
        m,_,_,_,provenance=assets('active');original_flags=int(m.opt.disableflags)
        if variant=='legacy_MPR':m.opt.disableflags=original_flags|bit
        if variant=='no_midphase':m.opt.disableflags=original_flags|int(mujoco.mjtDisableBit.mjDSBL_MIDPHASE)
        if variant.startswith('ccd'):m.opt.ccd_iterations=int(variant[3:])
        cube=m.geom('cube').id
        for relative in cases:
            path=OUT/relative;z=dict(np.load(path,allow_pickle=False))
            if 'qvel' not in z:
                base=dict(np.load(OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation/end_state.npz',allow_pickle=False));base.update(z);base['qvel'][:]=0;base['qacc_warmstart'][:]=0;z=base
            d=restore(m,z);dist=[]
            for name in ('rf_tip','th_tip','if_tip'):
                gid=m.geom(name).id;pair=np.zeros(6);native_distance=mujoco.mj_geomDistance(m,d,cube,gid,.1,pair)
                contacts=[dict(distance_mm=float(c.dist)*1000,exclude=int(c.exclude)) for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,gid}]
                dist.append(dict(geom=name,native_distance_mm=float(native_distance)*1000,contacts=contacts))
            rows.append(dict(variant=variant,disableflags=int(m.opt.disableflags),ccd_iterations=int(m.opt.ccd_iterations),native_ccd_enabled=not bool(int(m.opt.disableflags)&bit),source=relative,source_sha256=sha(path),distances=dist))
        for name in ('rf_tip','th_tip','if_tip','cube'):
            gid=m.geom(name).id;mid=int(m.geom_dataid[gid]);adr=int(m.mesh_vertadr[mid]);v=np.asarray(m.mesh_vert[adr:adr+int(m.mesh_vertnum[mid])],float)
            if int(m.geom_type[gid])==6:v=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*m.geom_size[gid]
            if variant=='original':geometries.append(dict(geom=name,computed_vertices_min=v.min(0).tolist(),computed_vertices_max=v.max(0).tolist(),
                compiled_geom_aabb=m.geom_aabb[gid].tolist(),rbound=float(m.geom_rbound[gid]),
                body_bvh_address=int(m.body_bvhadr[int(m.geom_bodyid[gid])]),body_bvh_count=int(m.body_bvhnum[int(m.geom_bodyid[gid])]),
                body_bvh_aabb=m.bvh_aabb[int(m.body_bvhadr[int(m.geom_bodyid[gid])])].tolist()))
    result=dict(status='separate_CCD_algorithm_diagnostic',native_ccd_disable_bit=bit,cases=rows,compiled_geometry_bounds=geometries,
        original_model_provenance=provenance,original_model_file_modified=False,model_flag_changed_only_in_diagnostic_copy=True,
        original_native_ccd_was_already_enabled=True,
        superseded_label_correction='The first diagnostic mislabeled the original as MPR and clearing an already-clear bit changed nothing. This comparison explicitly sets the legacy bit and separately disables midphase in copies.',
        geometry_or_joint_limits_or_acceptance_thresholds_changed=False,
        this_does_not_transfer_previous_policy_validation_to_the_variant=True,devices_opened=0,hardware_execution_allowed=False)
    save_json(OUT/'next_cycle/handoff_refinement_20261110/CCD_comparison.json',result);print(json.dumps(result),flush=True)


if __name__=='__main__':main()
