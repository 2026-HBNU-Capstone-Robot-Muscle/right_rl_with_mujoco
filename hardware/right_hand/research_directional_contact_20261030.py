"""Simulation-only normal-contact correction of the existing counter-turn.

Use the evolved forward checkpoint. The Cube remains a free body throughout.
No runtime controller or hardware admission is provided by this research file.
"""
import argparse,json,hashlib,math
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
R=Path(__file__).resolve().parent;BASE=R/'right_hand_rotation_20261030/directional_contact_research';CP=BASE/'checkpoint_nominal'
def motor(q):
 v=np.empty(16,int)
 for i,j in enumerate(SIM_TO_MOTOR_IDS):v[j-1]=int(round(q[i]/TICK+2048))
 return v
def support_local(m,g,n):
 kind=int(m.geom_type[g]);size=m.geom_size[g]
 if kind==6:return -np.where(abs(n)>1e-9,np.sign(n),0)*size
 if kind==3:
  p=-size[0]*n;p[2]-=np.sign(n[2])*size[1];return p
 if kind==2:return -size[0]*n
 if kind==4:return -size**2*n/np.linalg.norm(size*n)
 if kind==7:
  k=int(m.geom_dataid[g]);a=int(m.mesh_vertadr[k]);v=m.mesh_vert[a:a+int(m.mesh_vertnum[k])];return v[np.argmin(v@n)].copy()
 raise RuntimeError('Unsupported contact geometry; no proxy shape substituted')
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--offset-limit',type=float,required=True);a=ap.parse_args();cap=a.offset_limit;assert cap in (0.,.06,.12)
 O=BASE/f'normal_gap_cap_{cap:g}';O.mkdir(parents=True,exist_ok=True)
 m=mujoco.MjModel.from_binary_path(str(CP/'forward_checkpoint.mjb'));assert abs(m.opt.timestep-.001)<1e-12
 d=mujoco.MjData(m);s=np.load(CP/'forward_checkpoint.npz')
 for k in ('qpos','qvel','act','ctrl','qacc_warmstart'):getattr(d,k)[:]=s[k]
 d.time=float(s['time']);mujoco.mj_forward(m,d);virtual=mujoco.MjData(m);up=s['up'];cube=m.geom('cube').id;cube_body=m.body('cube').id
 B=d.xmat[cube_body].reshape(3,3).copy();hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 supports={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,supports)
 lo,hi=limits();simids=np.array(SIM_TO_MOTOR_IDS)-1;nlo=(lo[simids]-2048)*TICK;nhi=(hi[simids]-2048)*TICK
 original=np.load(R/'right_hand_rotation_20261030/yaw_roundtrip/candidate.npz');end=int(original['forward_end_sample']);previous=motor(d.ctrl);assert np.array_equal(previous,original['goals'][end-1])
 tips=[m.geom(n).id for n in ('mf_tip','th_tip')];axes=[]
 for g in tips:
  cs=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}]
  assert cs,'Evolved forward checkpoint has no requested fingertip contact'
  c=min(cs,key=lambda c:c.dist);outward=c.frame[:3]*(1 if int(c.geom1)==cube else -1);local=B.T@outward;k=int(np.argmax(abs(local)));assert abs(local[k])>.9
  axis=np.zeros(3);axis[k]=np.sign(local[k]);axes.append(axis)
 active=np.array([6,7,14,15]);jacp=np.empty((3,m.nv));jacr=np.empty_like(jacp)
 def gaps(q):
  virtual.qpos[:]=d.qpos;virtual.qpos[:16]=q;virtual.qvel[:]=0;mujoco.mj_kinematics(m,virtual);mujoco.mj_comPos(m,virtual)
  cubeR=virtual.xmat[cube_body].reshape(3,3);C=virtual.geom_xpos[cube];values=[];J=[];extent=[]
  for g,axis in zip(tips,axes):
   n=cubeR@axis;Gr=virtual.geom_xmat[g].reshape(3,3);p=virtual.geom_xpos[g]+Gr@support_local(m,g,Gr.T@n)
   values.append(float(n@(p-C)-.0275));mujoco.mj_jac(m,virtual,jacp,jacr,p,int(m.geom_bodyid[g]));J.append(n@jacp[:,active]);extent.append((cubeR.T@(virtual.geom_xpos[g]-C)).tolist())
  return np.array(values),np.array(J),extent
 targets,_,_=gaps(d.ctrl);initial_force=m.actuator_forcerange.copy();initial_gains=m.actuator_gainprm.copy();initial_bias=m.actuator_biasprm.copy()
 source_review=json.loads((R/'right_hand_rotation_20261030/yaw_roundtrip/full_loaded_seated_front_-10/review.json').read_bytes());C0=np.array(next(c['initial_cube_after_settle'] for c in source_review['cases'] if c['gain_scale']==1. and c['friction']==.3))
 qs=[];ph=[];trace=[];maxself=maxsup=maxcp=maxtrack=maxrange=longest=gap_seconds=0.;external=False;stopped=None;forces=np.zeros(16);max_goal_offset=np.zeros(16);counter_angle=None
 def angle():
  delta=Rotation.from_matrix(d.xmat[cube_body].reshape(3,3)@B.T).as_quat();return float(np.degrees(2*np.arctan2(delta[:3]@up,delta[3])))
 def step(q,phase,source_index):
  nonlocal previous,maxself,maxsup,maxcp,maxtrack,maxrange,longest,gap_seconds,external,stopped,forces
  if np.max(abs(q-previous))>2 or np.any(q<lo) or np.any(q>hi):stopped='goal range/step';return False
  previous=q.copy();d.ctrl[:]=ticks_to_sim(q);qs.append(q.copy());ph.append(phase)
  for _ in range(75):
   mujoco.mj_step(m,d);su,sp=inspect(d);maxself=max(maxself,max(sp.values(),default=0.));maxsup=max(maxsup,max(su.values(),default=0.));maxtrack=max(maxtrack,float(max(abs(d.qpos[:16]-d.ctrl))));maxrange=max(maxrange,float(max(np.maximum(nlo-d.qpos[:16],d.qpos[:16]-nhi))));forces=np.maximum(forces,abs(d.actuator_force));inhand=False
   for c in d.contact:
    aa,bb=int(c.geom1),int(c.geom2)
    if cube not in (aa,bb):continue
    other=bb if aa==cube else aa
    if int(m.geom_bodyid[other]) in hand:maxcp=max(maxcp,max(0.,-float(c.dist)*1000));inhand|=c.dist<.0004
    elif other in supports and c.dist<.0004:external=True
   gap_seconds=0. if inhand else gap_seconds+.001;longest=max(longest,gap_seconds)
   if maxself>1e-6 or maxsup>.15 or maxcp>1 or maxtrack>.18 or maxrange>1e-6 or external or longest>.25 or np.linalg.norm(d.qpos[-7:-4]-C0)>.08:
    stopped='unchanged contact/range/tracking/retention bound';break
  if len(qs)%10==0 or stopped:
   loads={}
   for ci,c in enumerate(d.contact):
    aa,bb=int(c.geom1),int(c.geom2)
    if cube not in (aa,bb):continue
    other=bb if aa==cube else aa;f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);loads[m.geom(other).name]=loads.get(m.geom(other).name,0.)+float(f[0])
   trace.append(dict(sample=len(qs),source_index=source_index,phase=phase,inverse_angle_deg=angle(),tip_force_N={m.geom(g).name:loads.get(m.geom(g).name,0.) for g in tips},all_cube_contact_normal_forces_N=loads,ctrl=q.tolist()))
  return stopped is None
 inverse=original['goals'][end:][original['phases'][end:]=='stage_7_horizontal_yaw'];assert len(inverse)==638
 for i,base in enumerate(inverse):
  target=base.copy()
  if cap:
   bq=ticks_to_sim(base);guess=ticks_to_sim(previous)
   guess[np.setdiff1d(np.arange(16),active)]=bq[np.setdiff1d(np.arange(16),active)]
   for _ in range(4):
    error,J,_=gaps(guess);dq=np.linalg.lstsq(np.vstack([J,.001*np.eye(4)]),np.r_[targets-error,np.zeros(4)],rcond=None)[0]
    guess[active]=np.clip(guess[active]+dq,np.maximum(nlo[active],bq[active]-cap),np.minimum(nhi[active],bq[active]+cap))
   desired=motor(guess);target=previous+np.clip(desired-previous,-2,2);max_goal_offset=np.maximum(max_goal_offset,abs(target-base))
  if not step(target,'stage_7_horizontal_yaw',i+1):break
 if stopped is None:
  for i in range(40):
   if not step(previous.copy(),'stage_8_horizontal_hold',638):break
  counter_angle=angle()
 if stopped is None:
  grip=original['goals'][end+638-1];start=previous.copy();n=max(1,int(np.ceil(max(abs(grip-start))/2)))
  for i in range(1,n+1):
   if not step(np.rint(start+(grip-start)*i/n).astype(int),'stage_9_loaded_open',None):break
 if stopped is None:
  mask=np.isin(original['phases'],['stage_9_loaded_open','stage_10_remove_hold'])
  for q,p in zip(original['goals'][mask],original['phases'][mask]):
   if not step(q,str(p),None):break
 assert np.array_equal(m.actuator_forcerange,initial_force) and np.array_equal(m.actuator_gainprm,initial_gains) and np.array_equal(m.actuator_biasprm,initial_bias)
 passed=stopped is None;new=np.vstack([original['goals'][:end],qs]);phases=np.array(original['phases'][:end].tolist()+ph)
 np.savez_compressed(O/('candidate.npz' if passed else 'rejected_partial_goals.npz'),start=original['start'],goals=new,phases=phases,forward_end_sample=np.array(end))
 result=dict(status='simulation_only_directional_normal_gap_research',offset_limit_rad=cap,passed_unchanged_safety_bounds=passed,stop=stopped,modeled_inverse_at_hold_deg=counter_angle,modeled_inverse_after_open_deg=angle(),target_rotation_met=bool(counter_angle is not None and abs(counter_angle+90)<10),
  initial_virtual_gap_targets_mm=(targets*1000).tolist(),preserves_initial_virtual_gap_not_increased_force_setting=True,uses_simulation_object_state_not_live_camera=True,actual_cube_pose_not_pinned=True,
  maximum_goal_offset_ticks=max_goal_offset.tolist(),max_self_mm=maxself,max_support_mm=maxsup,max_cube_mm=maxcp,max_tracking_rad=maxtrack,unapproved_range_excess_rad=max(0,maxrange),longest_contact_gap_s=longest,external_cube_contact=external,
  peak_actuator_force_model_units=forces.tolist(),force_limits_and_gains_unchanged=True,proposed_whole_loaded_goals=len(new),nominal_whole_loaded_s=len(new)*.075,
  hardware_execution_allowed=False,new_physical_rotation_not_verified=True,devices_opened=False,trace=trace)
 (O/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in result.items() if k!='trace'}),flush=True)
if __name__=='__main__':main()
