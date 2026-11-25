"""Offline convex-hull distance check for the paired physical-contact pose."""
import json, itertools
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial import ConvexHull
from scipy.optimize import minimize, LinearConstraint
from mujoco_playground import registry
from hand_motor_mapping import ticks_to_sim
from box_clearance import clearance
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'full_mapping_pose_review_20261019'

def convex_distance(a,b):
    # Work in millimetres near origin to avoid poorly scaled constraints.
    origin=(np.mean(a,axis=0)+np.mean(b,axis=0))/2
    a=(a-origin)*1000;b=(b-origin)*1000
    ha,hb=ConvexHull(a),ConvexHull(b)
    A=np.zeros((len(ha.equations)+len(hb.equations),6))
    A[:len(ha.equations),:3]=ha.equations[:,:3]
    A[len(ha.equations):,3:]=hb.equations[:,:3]
    upper=-np.r_[ha.equations[:,3],hb.equations[:,3]]
    def fun(x):return .5*float(np.sum((x[:3]-x[3:])**2))
    def jac(x):return np.r_[x[:3]-x[3:],x[3:]-x[:3]]
    result=minimize(fun,np.r_[a.mean(0),b.mean(0)],jac=jac,method='SLSQP',
        constraints=[LinearConstraint(A,-np.inf,upper)],options=dict(ftol=1e-10,maxiter=500))
    violation=float(np.max(A@result.x-upper))
    if not result.success or violation>1e-5:raise RuntimeError((result.message,violation))
    # Dual geometric certificate: separating plane perpendicular to the closest
    # pair gives a lower bound. The feasible pair gives an upper bound.
    delta=result.x[3:]-result.x[:3];distance=float(np.linalg.norm(delta))
    axis=delta/distance if distance>1e-8 else np.zeros(3)
    lower=max(0.,float(np.min(b@axis)-np.max(a@axis)))
    if distance-lower>1e-4:raise RuntimeError(('uncertified distance gap',distance,lower))
    return dict(gap_mm=distance,certified_lower_bound_mm=lower,
                feasible_violation_mm=max(0.,violation),closest_a_m=(result.x[:3]/1000+origin).tolist(),
                closest_b_m=(result.x[3:]/1000+origin).tolist())

# Independent analytic/OBB checks before using the solver on meshes.
signs=np.array(list(itertools.product((-1,1),repeat=3)))
for angle,shift in ((0,[.03,0,0]),(.4,[.04,.02,.01]),(.7,[0,0,0])):
    R=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
    a=signs*.01;b=(signs*np.array([.012,.008,.006]))@R.T+shift
    expected,_=clearance(np.zeros(3),np.eye(3),np.full(3,.01),np.array(shift),R,np.array([.012,.008,.006]))
    assert abs(convex_distance(a,b)['gap_mm']-1000*expected)<1e-4

cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
m=registry.load('LeapCubeFaceUp',config=cfg).mj_model;d=mujoco.MjData(m)
source=ROOT/'pose_pair_readonly_20261017_015118_321695/result.json'
r=json.loads(source.read_text(encoding='utf-8'))
raw=np.mean([[v['position_ticks_signed'] for v in sorted(s['motors'],key=lambda v:v['id'])] for s in r['snapshots']],axis=0)
d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
def vertices(g):
    mesh=int(m.geom_dataid[g]);first=int(m.mesh_vertadr[mesh]);num=int(m.mesh_vertnum[mesh])
    return np.array(m.mesh_vert[first:first+num])@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
middle=[g for g in range(m.ngeom) if m.geom_type[g]==7 and m.body(int(m.geom_bodyid[g])).name.startswith('mf_')]
thumb=[g for g in range(m.ngeom) if m.geom_type[g]==7 and m.body(int(m.geom_bodyid[g])).name in ('th_px','th_ds')]
rows=[]
for a in middle:
    for b in thumb:
        row=dict(a=m.geom(a).name,b=m.geom(b).name,**convex_distance(vertices(a),vertices(b)));rows.append(row)
        print(json.dumps({k:row[k] for k in ('a','b','gap_mm','certified_lower_bound_mm')}),flush=True)
report=dict(status='offline_visual_mesh_convex_hull_check',motor_commands_sent=0,
            zero_offsets_fitted=False,geometry_calibration_verified=False,solver_checks_passed=3,
            approximation='Convex hulls of rendered meshes. Hull gap is a lower bound on the nonconvex mesh separation; overlap does not establish real contact.',
            source=str(source.relative_to(ROOT)),rows=rows,nearest=min(rows,key=lambda r:r['gap_mm']))
(OUT/'contact_mesh_clearance.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
