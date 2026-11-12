"""Retarget the two-finger reference to the free-settled recess model.

Research only: task-frame change inferred from saved frames is a hypothesis.
No force/gain/collision/range changes and no hardware program are installed.
"""
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
R=Path(__file__).resolve().parent;BASE=R/'right_hand_rotation_20261030/stable_support_research';O=BASE/'retargeted_forward';CP=R/'right_hand_rotation_20261030/directional_contact_research/checkpoint_nominal'
def motor(q):
 out=np.empty(16,int)
 for i,j in enumerate(SIM_TO_MOTOR_IDS):out[j-1]=round(q[i]/TICK+2048)
 return out
def blend(a,b):
 n=max(1,int(np.ceil(max(abs(b-a))/2)));return np.rint([a+(b-a)*k/n for k in range(1,n+1)]).astype(int)
def main():
 O.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CP/'forward_checkpoint.mjb'));old=np.load(BASE/'previous_reference_seed_state.npz');new=np.load(BASE/'natural_recess_hypothesis_state.npz');source=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');d=mujoco.MjData(m);ref=mujoco.MjData(m)
 for target,s in ((d,new),(ref,old)):
  target.qpos[:]=s['qpos'];target.qvel[:]=0;target.ctrl[:]=s['ctrl'];mujoco.mj_forward(m,target)
 cube=m.body('cube').id;B0=ref.xmat[cube].reshape(3,3).copy();B1=d.xmat[cube].reshape(3,3).copy();C0=ref.xpos[cube].copy();C1=d.xpos[cube].copy();A=B1@B0.T;tips=[m.geom(n).id for n in ('mf_tip','th_tip')];active=np.array([4,5,6,7,12,13,14,15]);lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1;ql=(np.ceil(lo[ids])-2048)*TICK;qh=(np.floor(hi[ids])-2048)*TICK;hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 supports={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,supports)
 end=int(source['forward_end_sample']);goals=[];phases=[];previous=source['start'].copy();correction=np.zeros(8);worst=dict(point_error_mm=0.,self_mm=0.,support_mm=0.);failed=None;records=[]
 for index,(phase,base) in enumerate(zip(source['phases'][:end],source['goals'][:end]),1):
  q=ticks_to_sim(base);ref.qpos[:16]=q;ref.ctrl[:]=q;mujoco.mj_forward(m,ref);p0=np.array([ref.geom_xpos[g].copy() for g in tips]);fraction=min(1.,index/250.);alpha=fraction*fraction*(3-2*fraction);targets=p0+alpha*((p0-C0)@A.T+C1-p0)
  def state(x):
   test=q.copy();test[active]=x;d.qpos[:16]=test;d.ctrl[:]=test;mujoco.mj_forward(m,d);errors=np.array([d.geom_xpos[g]-p for g,p in zip(tips,targets)]);su,sp=inspect(d);return errors,max(sp.values(),default=0.),max(su.values(),default=0.),test
  def residual(x):
   errors,sm,sp,test=state(x);return np.r_[errors.ravel()*1000,(x-q[active])*.02,sm*500,sp*500]
  guess=np.clip(q[active]+correction,ql[active]+1e-10,qh[active]-1e-10);opt=least_squares(residual,guess,bounds=(ql[active],qh[active]),max_nfev=60,ftol=1e-8,xtol=1e-8,gtol=1e-8);errors,sm,sp,test=state(opt.x);err=float(max(np.linalg.norm(errors,axis=1))*1000);target=motor(test);correction=opt.x-q[active]
  worst={k:max(worst[k],v) for k,v in dict(point_error_mm=err,self_mm=sm,support_mm=sp).items()};good=err<.5 and sm<=1e-6 and sp<=.15 and np.all(target>=lo) and np.all(target<=hi)
  if not good:failed=dict(source_sample=index,phase=str(phase),point_error_mm=err,self_mm=sm,support_mm=sp);break
  rows=blend(previous,target);goals.extend(rows);phases.extend([str(phase)]*len(rows));previous=target
  if index%100==0 or index==end:
   record=dict(source_sample=index,retargeted_sample=len(goals),point_error_mm=err,max_goal_change_ticks=int(max(abs(target-base))),max_self_mm=sm,max_support_mm=sp);records.append(record);print(json.dumps(record),flush=True)
 if failed is None:
  v=np.asarray(goals);assert max(abs(np.diff(np.vstack([source['start'],v]),axis=0)).ravel())<=2;assert np.array_equal(v[-1][np.array([4,5,6,7,12,13,14,15])],source['goals'][end-1][np.array([4,5,6,7,12,13,14,15])]);np.savez_compressed(O/'candidate.npz',start=source['start'],goals=v,phases=np.array(phases),forward_end_sample=np.array(len(v)))
 report=dict(status='natural_recess_forward_task_frame_retarget_research_only',geometry_passed=failed is None,failed=failed,model_settled_center_change_mm=((C1-C0)*1000).tolist(),model_settled_cube_rotation_change_deg=float(np.degrees(np.arccos(np.clip((np.trace(A)-1)/2,-1,1)))),active_physical_ids=[SIM_TO_MOTOR_IDS[i] for i in active],index_ring_targets_preserved=True,task_frame_correction_blended_over_first250_source_samples=True,source_goals=end,retargeted_goals=len(goals),worst=worst,records=records,source_sha256=hashlib.sha256((R/'right_hand_rotation_20261030/single_forward/candidate.npz').read_bytes()).hexdigest(),hardware_execution_allowed=False,dynamics_not_yet_verified=True,marker_to_robot_registration_unverified=True,motor_writes=0)
 (O/'geometry_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='records'}),flush=True)
if __name__=='__main__':main()
