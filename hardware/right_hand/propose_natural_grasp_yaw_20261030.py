"""Design a full90 task from the evolved new grasp. Offline geometry only."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares,brentq
from scipy.spatial.transform import Rotation
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
R=Path(__file__).resolve().parent;BASE=R/'right_hand_rotation_20261030/natural_grasp_yaw';CP=BASE/'checkpoint'
def motor(q):
 v=np.empty(16,int)
 for i,j in enumerate(SIM_TO_MOTOR_IDS):v[j-1]=round(q[i]/TICK+2048)
 return v
def blend(a,b):
 n=max(1,int(np.ceil(max(abs(b-a))/2)));return np.rint([a+(b-a)*i/n for i in range(1,n+1)]).astype(int)
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--center',choices=('fixed','free'),required=True);ap.add_argument('--direction',choices=(-1,1),type=int,default=1);ap.add_argument('--center-cost',type=float,default=200.);ap.add_argument('--normal-cost',type=float,default=2.);args=ap.parse_args();assert args.center_cost>0 and args.normal_cost>0;name=('positive' if args.direction==1 else 'negative')+'_'+args.center
 if (args.center_cost,args.normal_cost)!=(200.,2.):name+=f'_c{args.center_cost:g}_n{args.normal_cost:g}'
 O=BASE/name;O.mkdir(exist_ok=True)
 m=mujoco.MjModel.from_binary_path(str(CP/'grasp_checkpoint.mjb'));s=np.load(CP/'grasp_checkpoint.npz');d=mujoco.MjData(m);d.qpos[:]=s['qpos'];d.qvel[:]=0;d.ctrl[:]=s['ctrl'];mujoco.mj_forward(m,d);q0=d.qpos[:16].copy();pd=s['ctrl']-q0;cube=m.geom('cube').id;cb=m.body('cube').id;B0=d.xmat[cb].reshape(3,3).copy();C0=d.geom_xpos[cube].copy();Binit=s['initial_cube_rotation'];up=s['up'];axis=-B0[:,1];assert axis@up>np.cos(np.radians(15));tips=[m.geom(n).id for n in ('mf_tip','th_tip')];local=[];points=[];normals=[];ln=[]
 for g in tips:
  contacts=[(i,c) for i,c in enumerate(d.contact) if {int(c.geom1),int(c.geom2)}=={cube,g}]
  def force(item):
   f=np.zeros(6);mujoco.mj_contactForce(m,d,item[0],f);return f[0]
  ci,c=max(contacts,key=force);G=d.geom_xmat[g].reshape(3,3);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);points.append(c.pos.copy());normals.append(n.copy());local.append(G.T@(c.pos-d.geom_xpos[g]));ln.append(G.T@n)
 def twist(B):
  qq=Rotation.from_matrix(B@Binit.T).as_quat();return float(np.degrees(2*np.arctan2(qq[:3]@up,qq[3])))
 target=90.*args.direction;bracket=(60.,120.) if args.direction==1 else (-120.,-60.);end=brentq(lambda degrees:twist(Rotation.from_rotvec(axis*np.radians(degrees)).as_matrix()@B0)-target,*bracket)
 active=np.array([4,5,6,7,12,13,14,15]);lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1;low=(np.ceil(lo[ids])-2048)*TICK;high=(np.floor(hi[ids])-2048)*TICK;lower=np.maximum(m.jnt_range[:16,0][active],low[active]-pd[active]);upper=np.minimum(m.jnt_range[:16,1][active],high[active]-pd[active]);hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);basis=np.column_stack([B0[:,0],B0[:,2],axis]);center_free=args.center=='free';x=q0[active].copy();x=np.r_[x,[0.,0.,0.]] if center_free else x
 if center_free:lower=np.r_[lower,[-.015,-.015,-.002]];upper=np.r_[upper,[.015,.015,.015]]
 previous=motor(s['ctrl']);goals=[];records=[];failure=None
 for theta in np.linspace(0.,end,int(np.ceil(abs(end)))+1):
  D=Rotation.from_rotvec(axis*np.radians(theta)).as_matrix();B=D@B0;quat=Rotation.from_matrix(B).as_quat();target_normals=[D@n for n in normals]
  def metrics(value):
   q=q0.copy();q[active]=value[:8];C=C0+(basis@value[8:] if center_free else 0.);d.qpos[:]=s['qpos'];d.qpos[:16]=q;d.qpos[-7:-4]=C;d.qpos[-4:]=np.r_[quat[3],quat[:3]];d.qvel[:]=0;d.ctrl[:]=q+pd;mujoco.mj_forward(m,d);errors=np.array([d.geom_xpos[g]+d.geom_xmat[g].reshape(3,3)@p-(C+D@(p0-C0)) for g,p,p0 in zip(tips,local,points)]);ne=np.array([d.geom_xmat[g].reshape(3,3)@n-tn for g,n,tn in zip(tips,ln,target_normals)]);su,sp=inspect(d);cp=0.
   for c in d.contact:
    if cube in (int(c.geom1),int(c.geom2)):cp=max(cp,max(0.,-float(c.dist)*1000))
   return errors,ne,max(sp.values(),default=0.),max(su.values(),default=0.),cp,q,C
  old=x.copy()
  def residual(value):
   e,ne,sm,sp,cp,q,C=metrics(value);v=np.r_[e.ravel()*1000,ne.ravel()*args.normal_cost,(value[:8]-old[:8])*.05,sm*500,sp*500,max(0.,cp-.8)*80]
   return np.r_[v,value[8:]*args.center_cost] if center_free else v
  opt=least_squares(residual,np.clip(x,lower+1e-10,upper-1e-10),bounds=(lower,upper),max_nfev=180,ftol=1e-9,xtol=1e-9,gtol=1e-9);x=opt.x;e,ne,sm,sp,cp,q,C=metrics(x);err=np.linalg.norm(e,axis=1)*1000;normal_angles=np.degrees(np.arccos(np.clip(1-np.sum(ne*ne,axis=1)/2,-1,1)));goal=motor(q+pd);passed=bool(max(err)<.5 and sm<=1e-6 and sp<=.15 and cp<=1 and np.all(goal>=lo) and np.all(goal<=hi));row=dict(planned_delta_deg=float(theta),planned_whole_yaw_deg=twist(B),contact_point_errors_mm=err.tolist(),normal_angle_errors_deg=normal_angles.tolist(),center_shift_mm=((C-C0)*1000).tolist(),self_mm=sm,support_mm=sp,cube_mm=cp,passed=passed,goal=goal.tolist(),evaluations=opt.nfev);records.append(row)
  if len(records)%10==1 or not passed or abs(theta-end)<1e-6:print(json.dumps(row),flush=True)
  if not passed:failure='this local search did not find an acceptable contact/range/collision waypoint';break
  step=blend(previous,goal);goals.extend(step);previous=goal
 source=np.load(R/'right_hand_rotation_20261030/natural_cradle_grasp/candidate.npz');n=int(source['forward_end_sample']);assert n==927;name='candidate.npz' if failure is None else 'rejected_partial_path.npz';whole=np.vstack([source['goals'][:n],goals,np.tile(previous,(40,1))]);phases=np.array(source['phases'][:n].tolist()+['stage_7_horizontal_yaw']*len(goals)+['stage_8_horizontal_hold']*40);assert max(abs(np.diff(np.vstack([source['start'],whole]),axis=0)).ravel())<=2;np.savez_compressed(O/name,start=source['start'],goals=whole,phases=phases,forward_end_sample=np.array(len(whole)))
 report=dict(status='natural_grasp_full90_geometry_research_only',full_geometry_completed=failure is None,failure=failure,center_mode=args.center,center_cost=args.center_cost,normal_cost=args.normal_cost,desired_whole_yaw_deg=target,required_delta_from_grasp_deg=end,initial_grasp_yaw_deg=twist(B0),last_planned_whole_yaw_deg=records[-1]['planned_whole_yaw_deg'],source_grasp_prefix_goals=927,rotation_goals=len(goals),source_checkpoint_sha256=hashlib.sha256((CP/'grasp_checkpoint.npz').read_bytes()).hexdigest(),cube_pinned_only_for_geometry_design_not_dynamic_test=True,native_geometry_motor_limits_and_force_PID_unchanged=True,actual_free_cube_rotation_not_yet_verified=True,records=records,hardware_execution_allowed=False,motor_writes=0)
 (O/'geometry_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='records'}),flush=True)
if __name__=='__main__':main()
