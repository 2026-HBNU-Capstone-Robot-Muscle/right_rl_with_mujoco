"""Identify the remaining upper-layer witnesses at the actual final hold."""
from pathlib import Path
import json
import numpy as np
from native_cycle import OUT,assets,restore,Guard,save_json,sha
from upper_layer_separation_certificate_20261105 import SweepCertificate
from upper_layer_cylinder_clearance_20261105 import precompute,gaps

def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';source=base/'robust_mu025_late_TH_normal_release_v35/force0.8/end_state.npz';folder=base/'reposition_then_second_rotation_20261111/final_hold_space_audit_named'
    if folder.exists():raise RuntimeError('Preserve final space witness')
    m,initial,*_=assets('active_ccd50');m.geom_friction[:,0]=.25;d=restore(m,dict(np.load(source,allow_pickle=False)));g=Guard(m);cube=m.geom('cube').id;C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);cert=SweepCertificate(m,g.hand,.3);lower,bad=cert.query(d,C,B);screen=gaps(m,d,C,B,precompute(m,g.hand),True)
    r=dict(status='actual_reached_final_hold_upper_space_witness',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),minimum_certified_lower_mm=lower,certificate_bad_geoms=[m.geom(int(cert.ids[i])).name for i in bad],certificate_bad_indices=bad,screening_gaps_mm=[dict(geom=name,gap_mm=value) for name,value in sorted(screen,key=lambda t:t[1])[:12]],negative_lower_bound_not_a_penetration_measurement=True,devices_opened=0,hardware_execution_allowed=False);folder.mkdir();save_json(folder/'review.json',r);print(json.dumps(r),flush=True)

if __name__=='__main__':main()
