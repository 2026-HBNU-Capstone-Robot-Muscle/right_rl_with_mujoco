"""Regression check against compiled CAD shapes and a recorded overlap witness."""
import json
import numpy as np
import mujoco
from adapt_original_sequence_clearance_20261024 import build,OUT
from hand_motor_mapping import ticks_to_sim


def main():
    m,manifest=build();d=mujoco.MjData(m)
    proxies=[g for g in range(m.ngeom) if m.geom(g).name.startswith('adapt_mf_rf_')]
    assert len(proxies)==10
    checks=[]
    for g in proxies:
        assert m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH
        peers=[j for j in range(m.ngeom) if m.geom_bodyid[j]==m.geom_bodyid[g]
            and m.geom_group[j]==2 and m.geom_type[j]==mujoco.mjtGeom.mjGEOM_MESH and m.geom_dataid[j]==m.geom_dataid[g]]
        assert len(peers)==1
        j=peers[0]
        assert np.allclose(m.geom_pos[g],m.geom_pos[j],atol=1e-12)
        assert abs(abs(np.dot(m.geom_quat[g],m.geom_quat[j]))-1)<1e-12
        checks.append(dict(proxy=m.geom(g).name,source_visual=m.geom(j).name,
            mesh=m.mesh(int(m.geom_dataid[g])).name,compiled_type='mesh',same_compiled_transform=True))
    root=OUT.parent.parent
    prior=json.loads((root/'book_dynamic_yaw_trial_20261024_222132_806567/transfer_diagnostic.json').read_bytes())
    trial=json.loads((root/'book_dynamic_yaw_trial_20261024_222132_806567/result.json').read_bytes())
    witness=max(prior['middle_ring_housing_model_contacts'].values(),key=lambda r:r['depth_mm'])
    sample=trial['samples'][witness['sample_index']]
    d.qpos[:16]=ticks_to_sim(sample['actual']);d.qpos[-7:]=[10,10,10,1,0,0,0];mujoco.mj_forward(m,d)
    actual=[]
    for c in d.contact:
        if c.dist<0 and int(c.geom1) in proxies and int(c.geom2) in proxies:
            actual.append(dict(pair=[m.geom(int(c.geom1)).name,m.geom(int(c.geom2)).name],depth_mm=-float(c.dist)*1000))
    depth=max(r['depth_mm'] for r in actual)
    assert depth>3.,'Known middle/ring housing overlap was not detected'
    assert abs(depth-witness['depth_mm'])<1e-5,(depth,witness['depth_mm'])
    result=dict(mesh_checks=checks,known_recorded_overlap_reproduced=True,
        previous_witness_depth_mm=witness['depth_mm'],current_witness_depth_mm=depth,
        witness_sample_index=witness['sample_index'],witness_phase=sample['phase'],
        compiled_mesh_types_verified=True,physical_overlap_depth_measured=False,
        model_revision=manifest['middle_ring_mesh_model_revision'],hardware_commands_sent=0)
    (OUT/'compiled_mesh_regression_validation.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':main()
