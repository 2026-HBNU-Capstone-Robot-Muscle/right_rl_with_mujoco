"""Inspect compiled mesh hulls when full-vertex screening and contacts differ."""
import json
from pathlib import Path
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,save_json,sha
from convex_distance_check_20261104 import CheckedDistance


def main():
    m,_,_,_,provenance=assets('active');cube=m.geom('cube').id;guard=Guard(m)
    geom_rows=[]
    for name in ('cube','if_tip','th_tip','rf_tip','if_bs_collision_1'):
        gid=m.geom(name).id;mid=int(m.geom_dataid[gid]);r=dict(name=name,type=int(m.geom_type[gid]),
             body=m.body(int(m.geom_bodyid[gid])).name,contype=int(m.geom_contype[gid]),conaffinity=int(m.geom_conaffinity[gid]),
             margin=float(m.geom_margin[gid]),gap=float(m.geom_gap[gid]),dataid=mid)
        if int(m.geom_type[gid])==7:
            adr=int(m.mesh_graphadr[mid]);r.update(mesh_vertices=int(m.mesh_vertnum[mid]),mesh_faces=int(m.mesh_facenum[mid]),
                    hull_graph_address=adr,hull_graph_first32=m.mesh_graph[adr:adr+32].tolist() if adr>=0 else [])
        geom_rows.append(r)
    paths=['next_cycle/handoff_refinement_20261110/fixed_grip_search0/proposal.npz',
           'next_cycle/handoff_refinement_20261110/fixed_grip_search1/proposal.npz',
           'next_cycle/handoff_integer_load_first_v6/end_state.npz']
    rows=[]
    for relative in paths:
        path=OUT/relative;z=dict(np.load(path,allow_pickle=False))
        if 'qvel' not in z:
            source=dict(np.load(OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation/end_state.npz',allow_pickle=False))
            source.update(z);source['qvel'][:]=0;source['qacc_warmstart'][:]=0;z=source
        d=restore(m,z);geometry=CheckedDistance(m,cube,[m.geom(n).id for n in ('rf_tip','th_tip','if_tip')]);distances=[]
        for name in ('rf_tip','th_tip','if_tip'):
            gid=m.geom(name).id;pair=np.zeros(6);distance=float(mujoco.mj_geomDistance(m,d,cube,gid,.1,pair))
            contacts=[dict(distance_mm=float(c.dist)*1000,exclude=int(c.exclude),efc_address=int(c.efc_address)) for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,gid}]
            distances.append(dict(geom=name,full_vertex_SAT_gap_mm=geometry.gap(d,gid)*1000,native_distance_mm=distance*1000,
                                  native_witness_separation_mm=float(np.linalg.norm(pair[3:]-pair[:3])*1000),contacts=contacts))
        rows.append(dict(path=relative,sha256=sha(path),distances=distances))
    options={name:float(getattr(m.opt,name)) for name in ('ccd_tolerance','ccd_iterations','disableflags','enableflags') if hasattr(m.opt,name)}
    pairs=[dict(geom1=m.geom(int(x)).name,geom2=m.geom(int(y)).name) for x,y in zip(m.pair_geom1,m.pair_geom2) if cube in (int(x),int(y))]
    header=Path(mujoco.__file__).parent/'include/mujoco/mjmodel.h';excerpt=[]
    for line in header.read_text().splitlines():
        if 'mesh_graph' in line or 'mesh_vert' in line:excerpt.append(line.strip())
    result=dict(status='compiled_mesh_and_native_contact_audit',mujoco_version=mujoco.__version__,model_provenance=provenance,
        options=options,geoms=geom_rows,explicit_cube_contact_pairs=pairs,exclude_signatures=m.exclude_signature.tolist(),
        model_header_mesh_fields=excerpt,cases=rows,model_or_collision_flags_changed=False,devices_opened=0,hardware_execution_allowed=False)
    save_json(OUT/'next_cycle/handoff_refinement_20261110/native_mesh_contact_audit.json',result)
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
