"""Independent, certified positive distance for an existing box/convex mesh.

No geom or MuJoCo engine flags are modified. A separating-axis proof is required
before replacing a contradictory native distance. A convex QP then computes
surface witnesses; its Frank-Wolfe gap bounds optimization error.
"""
import json
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import minimize,Bounds,LinearConstraint
from scipy.spatial import ConvexHull
class CheckedDistance:
 def __init__(self,m,cube,tips):
  assert int(m.geom_type[cube])==6;self.m=m;self.cube=cube;self.cache={};self.corrections=[]
  for g in tips:
   kind=int(m.geom_type[g]);assert kind in (6,7)
   if kind==7:
    mesh=int(m.geom_dataid[g]);a=int(m.mesh_vertadr[mesh]);v=np.asarray(m.mesh_vert[a:a+int(m.mesh_vertnum[mesh])],float)
   else:v=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*m.geom_size[g]
   hull=ConvexHull(v);v=v[hull.vertices];hull=ConvexHull(v);edges=set()
   for tri in hull.simplices:
    for a,b in ((tri[0],tri[1]),(tri[1],tri[2]),(tri[2],tri[0])):edges.add(tuple(sorted((int(a),int(b)))))
   edge=np.array([v[b]-v[a] for a,b in edges]);edge/=np.linalg.norm(edge,axis=1)[:,None];eq=np.unique(np.round(hull.equations,12),axis=0);self.cache[g]=(v,eq,np.unique(np.round(edge,10),axis=0))
 def gap(self,d,g):
  v,eq,edges=self.cache[g];B=d.geom_xmat[self.cube].reshape(3,3);G=d.geom_xmat[g].reshape(3,3);world=v@G.T+(d.geom_xpos[g]-d.geom_xpos[self.cube]);cross=np.cross(B.T[:,None,:],(edges@G.T)[None,:,:]).reshape(-1,3);length=np.linalg.norm(cross,axis=1);cross=cross[length>1e-9]/length[length>1e-9,None];axes=np.vstack([B.T,eq[:,:3]@G.T,cross]);r=np.abs(axes@B)@self.m.geom_size[self.cube];p=world@axes.T;return float(max(np.max(p.min(0)-r),np.max(-r-p.max(0))))
 def distance(self,d,g):
  pair=np.zeros(6);native=float(mujoco.mj_geomDistance(self.m,d,self.cube,g,.2,pair));sat=self.gap(d,g)
  if sat<=1e-8 or native>=sat-1e-7:return native,pair
  v,eq,edges=self.cache[g];B=d.geom_xmat[self.cube].reshape(3,3);G=d.geom_xmat[g].reshape(3,3);C=d.geom_xpos[self.cube];T=d.geom_xpos[g];M=np.column_stack([B,-G]);offset=(C-T)*1000;size=self.m.geom_size[self.cube]*1000;verts=v*1000;low=np.r_[-size,verts.min(0)];high=np.r_[size,verts.max(0)];x0=np.r_[B.T@(pair[:3]-C)*1000,G.T@(pair[3:]-T)*1000];x0=np.clip(x0,low,high);A=np.column_stack([np.zeros((len(eq),3)),eq[:,:3]]);ub=-eq[:,3]*1000
  def f(x):r=M@x+offset;return .5*float(r@r)
  def jac(x):return M.T@(M@x+offset)
  opt=minimize(f,x0,jac=jac,bounds=Bounds(low,high),constraints=[LinearConstraint(A,-np.inf,ub)],method='SLSQP',options=dict(ftol=1e-14,maxiter=250));x=opt.x
  # Polish numerical stationarity while preserving the convex feasible set.
  for _ in range(300):
   gradient=jac(x);best=np.r_[np.where(gradient[:3]>=0,low[:3],high[:3]),verts[np.argmin(verts@gradient[3:])]];direction=best-x;fw=-float(gradient@direction)
   if fw<=1e-6:break
   denominator=float((M@direction)@(M@direction))
   if denominator<1e-20:break
   x=x+min(1.,max(0.,fw/denominator))*direction
  gradient=jac(x);box_min=float(np.minimum(gradient[:3]*low[:3],gradient[:3]*high[:3]).sum());mesh_min=float(np.min(verts@gradient[3:]));fw=float(gradient@x-box_min-mesh_min);violation=max(0.,float(np.max(A@x-ub)),float(np.max(low-x)),float(np.max(x-high)));distance=np.linalg.norm(M@x+offset)/1000
  if not opt.success or fw>1e-5 or violation>1e-5 or distance<sat-1e-7:raise RuntimeError(f'Convex distance certificate failed: {opt.message}; gap={fw}; violation={violation}')
  corrected=np.r_[C+B@x[:3]/1000,T+G@x[3:]/1000];self.corrections.append(dict(native_m=native,sat_lower_bound_m=sat,corrected_m=float(distance),frank_wolfe_gap_mm2=fw,primal_violation_mm=violation));return float(distance),corrected
def main():
 R=Path(__file__).resolve().parent;root=R/'right_hand_rotation_20261030/natural_grasp_yaw';m=mujoco.MjModel.from_binary_path(str(root/'checkpoint/grasp_checkpoint.mjb'));d=mujoco.MjData(m);cube=m.geom('cube').id;tips=[m.geom(n).id for n in ('mf_tip','th_tip')];check=CheckedDistance(m,cube,tips);rows=[]
 from hand_motor_mapping import ticks_to_sim
 for name in ('contact_source_depth_1.5','contact_source_depth_1.5_lower_bodyheight'):
  diag=json.loads((root/name/'review.json').read_bytes())['diagnostic'];d.qpos[:16]=ticks_to_sim(diag['goal']);d.qpos[-7:]=diag['cube_pose'];d.qvel[:]=0;mujoco.mj_forward(m,d);g=m.geom(diag['geom']).id;dist,pair=check.distance(d,g);last=check.corrections[-1];assert dist>0 and abs(np.linalg.norm(pair[3:]-pair[:3])-dist)<1e-8;rows.append(dict(case=name,**last))
 report=dict(status='independent_convex_geometry_distance_validation',mujoco_version=mujoco.__version__,cases=rows,passed=len(rows)==2 and all(r['frank_wolfe_gap_mm2']<1e-5 for r in rows),source_issue='https://github.com/google-deepmind/mujoco/issues/3383',source_issue_matches_symptom_not_a_proven_engine_root_cause=True,model_geometries_and_collision_engine_unchanged=True,motor_writes=0);out=R/'right_hand_rotation_20261104';out.mkdir(exist_ok=True);(out/'convex_distance_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report))
if __name__=='__main__':main()
