"""Compare slab/radial screening with an independently rechecked support plane."""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import minimize,LinearConstraint,NonlinearConstraint,Bounds
from scipy.spatial import ConvexHull
from native_cycle import OUT,assets,restore,Guard,save_json,sha
from upper_layer_cylinder_clearance_20261105 import precompute,gaps,RADIUS,LOWER,UPPER
from upper_layer_separation_certificate_20261105 import SweepCertificate,cylinder_minimum


def main():
    path=OUT/'next_cycle/staged_model_y_plus05_fractional_encoders_admissible_correction/end_state.npz'
    m,_,_,_,provenance=assets('active');d=restore(m,dict(np.load(path,allow_pickle=False)));guard=Guard(m)
    cube=m.geom('cube').id;C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
    cert=SweepCertificate(m,guard.hand,.3);minimum,bad=cert.query(d,C,B);rows=[]
    screened=dict(gaps(m,d,C,B,precompute(m,guard.hand),True))
    for index in bad:
        gid,v,_=cert.geometry[index];G=d.geom_xmat[gid].reshape(3,3)
        vertices=(v@G.T+d.geom_xpos[gid]-C)@B*1000
        hull=ConvexHull(vertices);A=hull.equations[:,:3];b=-hull.equations[:,3]
        def objective(x):return .5*float(np.sum((x[:3]-x[3:])**2))
        def gradient(x):delta=x[:3]-x[3:];return np.r_[delta,-delta]
        def radial(x):return float(x[3]**2+x[5]**2)
        def radial_jac(x):return np.array([0.,0.,0.,2*x[3],0.,2*x[5]])
        center=vertices.mean(0);circle=center.copy();circle[1]=np.clip(circle[1],LOWER,UPPER)
        circle[[0,2]]*=min(1.,RADIUS/max(1e-9,np.linalg.norm(circle[[0,2]])))
        result=minimize(objective,np.r_[center,circle],jac=gradient,method='SLSQP',
            constraints=[LinearConstraint(np.c_[A,np.zeros_like(A)],-np.inf,b),
                         NonlinearConstraint(radial,-np.inf,RADIUS**2,jac=radial_jac)],
            bounds=Bounds([-np.inf]*4+[LOWER,-np.inf],[np.inf]*4+[UPPER,np.inf]),
            options=dict(ftol=1e-12,maxiter=2000))
        delta=result.x[3:]-result.x[:3];distance=float(np.linalg.norm(delta));normal=delta/max(distance,1e-15)
        gap=float(cylinder_minimum(normal)-max(vertices@normal))
        violation=max(0.,float(max(A@result.x[:3]-b)),float(radial(result.x)-RADIUS**2),
                      float(LOWER-result.x[4]),float(result.x[4]-UPPER))
        rows.append(dict(geom=m.geom(gid).name,slab_radial_screen_mm=screened[m.geom(gid).name],
            optimizer_success=bool(result.success),optimizer_message=str(result.message),
            feasible_pair_distance_upper_mm=distance if violation<1e-7 else None,
            feasible_pair_constraint_violation_mm_or_mm2=violation,
            exact_support_separation_lower_mm=gap,upper_minus_lower_mm=distance-gap,
            required_clearance_mm=.3,certified_over_required=gap>.3,
            inside_required_margin_upper_witness=violation<1e-7 and distance<.3))
    output=OUT/'next_cycle/upper_gap_metric_audit.json'
    save_json(output,dict(status='independent_cylinder_margin_audit',source_state=str(path.relative_to(OUT)),
        source_sha256=sha(path),model_provenance=provenance,previous_certificate_lower_mm=minimum,
        slab_radial_minimum_mm=min(screened.values()),failed_geoms=rows,
        interpretation='Slab-clipped radial screening detects intersection but is not a global Euclidean clearance lower bound near cylinder caps. Strict margin uses exact support planes.',
        geometry_changes=0,devices_opened=0,hardware_execution_allowed=False))
    print(json.dumps(dict(audit=str(output),failed_geoms=rows)),flush=True)


if __name__=='__main__':main()
