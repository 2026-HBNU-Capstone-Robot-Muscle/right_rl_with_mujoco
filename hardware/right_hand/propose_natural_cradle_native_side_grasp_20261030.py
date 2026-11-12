"""New side-contact grasp at the naturally supported cube; geometry research."""
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
R=Path(__file__).resolve().parent;BASE=R/'right_hand_rotation_20261030/stable_support_research';CP=R/'right_hand_rotation_20261030/directional_contact_research/checkpoint_nominal'
def motor(q):
 v=np.empty(16,int)
 for i,j in enumerate(SIM_TO_MOTOR_IDS):v[j-1]=round(q[i]/TICK+2048)
 return v
def blend(a,b):
 n=max(1,int(np.ceil(max(abs(b-a))/2)));return np.rint([a+(b-a)*k/n for k in range(1,n+1)]).astype(int)
def main():
 m=mujoco.MjModel.from_binary_path(str(CP/'forward_checkpoint.mjb'));s=np.load(CP/'forward_checkpoint.npz');natural=np.load(BASE/'natural_recess_hypothesis_state.npz');source=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');d=mujoco.MjData(m);ref=mujoco.MjData(m)
 ref.qpos[:]=s['qpos'];ref.qvel[:]=0;ref.ctrl[:]=s['ctrl'];mujoco.mj_forward(m,ref);cube=m.geom('cube').id;tips=[m.geom(n).id for n in ('mf_tip','th_tip')];local=[];local_normal=[]
 for g in tips:
  contacts=[(i,c) for i,c in enumerate(ref.contact) if {int(c.geom1),int(c.geom2)}=={cube,g}]
  def force(item):
   f=np.zeros(6);mujoco.mj_contactForce(m,ref,item[0],f);return f[0]
  ci,c=max(contacts,key=force);G=ref.geom_xmat[g].reshape(3,3);local.append(G.T@(c.pos-ref.geom_xpos[g]));n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);local_normal.append(G.T@n)
 d.qpos[:]=natural['qpos'];d.qvel[:]=0;d.ctrl[:]=natural['ctrl'];mujoco.mj_forward(m,d);B=d.xmat[m.body('cube').id].reshape(3,3).copy();C=d.geom_xpos[cube].copy();axis=max((0,2),key=lambda i:abs(B[:,i]@np.array([0,1,0])));n=B[:,axis]*np.sign(B[1,axis]);up=-B[:,1]
 active=np.array([4,5,6,7,12,13,14,15]);lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1;ql=(np.ceil(lo[ids])-2048)*TICK;qh=(np.floor(hi[ids])-2048)*TICK;graspq=source['goals'][np.where(source['phases']=='stage_6_grasp_hold')[0][-1]];guess=ticks_to_sim(graspq);pd=s['ctrl']-s['qpos'][:16];hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 supports={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,supports);records=[]
 for height in (0.,-.004,-.008):
  targets=np.array([C+n*.0275+up*height,C-n*.0275+up*height]);normals=[n,-n];lower=ql[active]-pd[active];upper=qh[active]-pd[active]
  def metrics(x):
   q=guess.copy();q[active]=x;d.qpos[:]=natural['qpos'];d.qpos[:16]=q;d.ctrl[:]=q+pd;mujoco.mj_forward(m,d);points=np.array([d.geom_xpos[g]+d.geom_xmat[g].reshape(3,3)@lp for g,lp in zip(tips,local)]);ne=np.array([d.geom_xmat[g].reshape(3,3)@ln-v for g,ln,v in zip(tips,local_normal,normals)]);su,sp=inspect(d);cp=0.
   for c in d.contact:
    if cube in (int(c.geom1),int(c.geom2)):cp=max(cp,max(0.,-float(c.dist)*1000))
   return points-targets,ne,max(sp.values(),default=0.),max(su.values(),default=0.),cp,q
  def residual(x):
   e,ne,sm,sp,cp,q=metrics(x);return np.r_[e.ravel()*1000,ne.ravel()*2.,(x-guess[active])*.05,sm*500,sp*500,max(0.,cp-.8)*80]
  opt=least_squares(residual,np.clip(guess[active],lower+1e-9,upper-1e-9),bounds=(lower,upper),max_nfev=250,ftol=1e-9,xtol=1e-9,gtol=1e-9);e,ne,sm,sp,cp,q=metrics(opt.x);errors=np.linalg.norm(e,axis=1)*1000;normal=np.degrees(np.arccos(np.clip(1-np.sum(ne*ne,axis=1)/2,-1,1)));goal=motor(q+pd);passed=max(errors)<.5 and sm<=1e-6 and sp<=.15 and cp<=1 and np.all(goal>=lo) and np.all(goal<=hi);folder=BASE/f'native_side_grasp_height_{int(height*1000)}';folder.mkdir(exist_ok=True)
  row=dict(height_mm=height*1000,geometry_passed=bool(passed),point_errors_mm=errors.tolist(),normal_angle_errors_deg=normal.tolist(),max_self_mm=sm,max_support_mm=sp,max_cube_mm=cp,goal=goal.tolist(),max_delta_from_previous_grasp_ticks=int(max(abs(goal-graspq))),source_control_offset_reused=True,hardware_execution_allowed=False,motor_writes=0);records.append(row);print(json.dumps(row),flush=True)
  if passed:
   close=blend(source['start'],goal);opening=blend(goal,source['start']);goals=np.vstack([close,np.tile(goal,(67,1)),opening,np.tile(source['start'],(20,1))]);phases=np.array(['stage_5_loaded_grasp']*len(close)+['stage_6_grasp_hold']*67+['stage_9_loaded_open']*len(opening)+['stage_10_remove_hold']*20);np.savez_compressed(folder/'candidate.npz',start=source['start'],goals=goals,phases=phases,forward_end_sample=np.array(len(close)+67));row.update(candidate_sha256=hashlib.sha256((folder/'candidate.npz').read_bytes()).hexdigest(),loaded_goals=len(goals),no_rotation_goals=True)
  (folder/'geometry_review.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
 (BASE/'native_side_grasp_geometry_comparison.json').write_text(json.dumps(dict(status='natural_cradle_native_range_side_contact_grasp_research',cases=records,source_cube_pinned_only_for_IK_not_dynamic_validation=True,original_geometry_forces_PID_native_limits_unchanged=True,hardware_execution_allowed=False,motor_writes=0),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
