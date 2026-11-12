"""Find feasible near/intersection witnesses for the RF hold's upper sweep."""
from pathlib import Path
import json
import numpy as np
from scipy.optimize import minimize,LinearConstraint,NonlinearConstraint,Bounds
from scipy.spatial import ConvexHull
from native_cycle import OUT,assets,restore,Guard,save_json,sha
from upper_layer_cylinder_clearance_20261105 import RADIUS,LOWER,UPPER
from upper_layer_separation_certificate_20261105 import SweepCertificate,cylinder_minimum


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/RF_upper_blockers'
    if folder.exists():raise RuntimeError('Preserve previous upper audit')
    source=OUT/'next_cycle/handoff_refinement_20261110/RF_endpoint_CCD50_load_screen/qualified0_end_state.npz'
    model,_,_,_,provenance=assets('active_ccd50');d=restore(model,dict(np.load(source,allow_pickle=False)))
    guard=Guard(model);cube=model.geom('cube').id;C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
    certificate=SweepCertificate(model,guard.hand,.3);lower,bad=certificate.query(d,C,B);rows=[]
    for index in bad:
        geom,vertices,_=certificate.geometry[index];G=d.geom_xmat[geom].reshape(3,3)
        points=(vertices@G.T+d.geom_xpos[geom]-C)@B*1000
        hull=ConvexHull(points);A=hull.equations[:,:3];b=-hull.equations[:,3]
        def objective(x):return .5*float(np.sum((x[:3]-x[3:])**2))
        def gradient(x):delta=x[:3]-x[3:];return np.r_[delta,-delta]
        def radial(x):return float(x[3]**2+x[5]**2)
        def radial_jac(x):return np.array([0.,0.,0.,2*x[3],0.,2*x[5]])
        center=points.mean(0);circle=center.copy();circle[1]=np.clip(circle[1],LOWER,UPPER)
        circle[[0,2]]*=min(1.,RADIUS/max(1e-12,np.linalg.norm(circle[[0,2]])))
        result=minimize(objective,np.r_[center,circle],jac=gradient,method='SLSQP',
            constraints=[LinearConstraint(np.c_[A,np.zeros_like(A)],-np.inf,b),
                         NonlinearConstraint(radial,-np.inf,RADIUS**2,jac=radial_jac)],
            bounds=Bounds([-np.inf]*4+[LOWER,-np.inf],[np.inf]*4+[UPPER,np.inf]),
            options=dict(ftol=1e-12,maxiter=1000))
        violation=max(0.,float(np.max(A@result.x[:3]-b)),float(radial(result.x)-RADIUS**2),
                      float(LOWER-result.x[4]),float(result.x[4]-UPPER))
        distance=float(np.linalg.norm(result.x[:3]-result.x[3:]))
        normal=(result.x[3:]-result.x[:3])/max(distance,1e-15)
        gap=float(cylinder_minimum(normal)-max(points@normal)) if distance>1e-10 else None
        row=dict(geom=model.geom(geom).name,body=model.body(int(model.geom_bodyid[geom])).name,
            optimizer_success=bool(result.success),constraint_violation_mm_or_mm2=violation,
            feasible_pair_distance_upper_mm=distance if violation<1e-7 else None,
            feasible_hand_point_cube_frame_mm=result.x[:3].tolist(),feasible_sweep_point_cube_frame_mm=result.x[3:].tolist(),
            witness_within_required_margin=bool(violation<1e-7 and distance<.3),
            exact_support_lower_bound_mm=gap,negative_certificate_value_is_not_penetration_depth=True)
        rows.append(row);print(json.dumps({key:row[key] for key in ('geom','feasible_pair_distance_upper_mm','witness_within_required_margin')}),flush=True)
    folder.mkdir();report=dict(status='RF_hold_upper_sweep_feasible_witness_audit',source=source.relative_to(OUT).as_posix(),
        source_sha256=sha(source),model_provenance=provenance,certificate_lower_bound_mm=lower,rows=rows,
        feasible_witness_confirms_required_space_failure=any(row['witness_within_required_margin'] for row in rows),
        original_model_and_guards_unchanged=True,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',report)
    (folder/'source_audit_RF_hold_upper_blockers.py').write_bytes(Path(__file__).read_bytes())


if __name__=='__main__':main()
