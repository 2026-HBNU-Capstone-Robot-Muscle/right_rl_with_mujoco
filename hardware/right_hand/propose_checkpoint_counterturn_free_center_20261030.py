"""Opposing contact-point IK from a freely evolved, saved forward endpoint.

Only design geometry uses a hypothetical Cube path. Dynamic validation must keep
the Cube free and must pass before this can become any execution proposal.
"""
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
R=Path(__file__).resolve().parent;BASE=R/'right_hand_rotation_20261030/directional_contact_research';CP=BASE/'checkpoint_nominal';O=BASE/'opposed_contact_free_center_ik'
def motor(q):
 a=np.empty(16,int)
 for i,j in enumerate(SIM_TO_MOTOR_IDS):a[j-1]=int(round(q[i]/TICK+2048))
 return a
def blend(a,b):
 n=max(1,int(np.ceil(max(abs(b-a))/2)));return np.rint([a+(b-a)*k/n for k in range(1,n+1)]).astype(int)
def main():
 O.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CP/'forward_checkpoint.mjb'));d=mujoco.MjData(m);s=np.load(CP/'forward_checkpoint.npz');d.qpos[:]=s['qpos'];d.qvel[:]=0;d.ctrl[:]=s['ctrl'];mujoco.mj_forward(m,d)
 q0=d.qpos[:16].copy();pd=s['ctrl']-q0;up=s['up'];cube=m.geom('cube').id;cb=m.body('cube').id;C0=d.xpos[cb].copy();B=d.xmat[cb].reshape(3,3).copy();tips=[m.geom(n).id for n in ('mf_tip','th_tip')]
 Pc=[];Pt=[];Nc=[];Nt=[]
 for g in tips:
  cs=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}];assert cs
  c=min(cs,key=lambda c:c.dist);point=c.pos.copy();n=c.frame[:3].copy()*(1 if int(c.geom1)==cube else -1);G=d.geom_xmat[g].reshape(3,3)
  Pc.append(B.T@(point-C0));Pt.append(G.T@(point-d.geom_xpos[g]));Nc.append(B.T@n);Nt.append(G.T@(-n))
 active=np.array([4,5,6,7,12,13,14,15]);ids=np.array(SIM_TO_MOTOR_IDS)-1;lo,hi=limits();nlo=(lo[ids]-2048)*TICK;nhi=(hi[ids]-2048)*TICK;clo=(np.ceil(lo[ids])-2048)*TICK;chi=(np.floor(hi[ids])-2048)*TICK
 e0=np.array([1.,0,0]);e0-=up*(e0@up);e0/=np.linalg.norm(e0);e1=np.cross(up,e0);basis=np.column_stack([e0,e1,up]);v=mujoco.MjData(m)
 hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support)
 start=np.r_[q0[active],[0.,0.,0.]];lower=np.r_[np.maximum(nlo[active],clo[active]-pd[active]),[-.02,-.02,-.003]];upper=np.r_[np.minimum(nhi[active],chi[active]-pd[active]),[.02,.02,.003]];assert np.all(start>=lower) and np.all(start<=upper)
 records=[];knots=[];failed=None
 def state(x,BR):
  q=q0.copy();q[active]=x[:8];C=C0+basis@x[8:];quat=np.empty(4);mujoco.mju_mat2Quat(quat,BR.ravel());v.qpos[:]=s['qpos'];v.qpos[:16]=q;v.qpos[-7:-4]=C;v.qpos[-4:]=quat;v.qvel[:]=0;v.ctrl[:]=q+pd;mujoco.mj_forward(m,v);return q,C
 for theta in np.arange(0,90.001,2):
  BR=Rotation.from_rotvec(-up*np.deg2rad(theta)).as_matrix()@B;prior=start.copy()
  def residual(x):
   q,C=state(x,BR);errors=[]
   for k,g in enumerate(tips):
    G=v.geom_xmat[g].reshape(3,3);P=v.geom_xpos[g]+G@Pt[k];errors.extend((P-(C+BR@Pc[k]))*1000);errors.extend((G@Nt[k]+BR@Nc[k])*20)
   su,sp=inspect(v);cube_pen=0.
   for c in v.contact:
    aa,bb=int(c.geom1),int(c.geom2)
    if cube in (aa,bb):cube_pen=max(cube_pen,max(0.,-float(c.dist)*1000))
   errors.extend([max(sp.values(),default=0.)*500,max(su.values(),default=0.)*500,max(0.,cube_pen-.5)*50])
   errors.extend(x[8:]*5);errors.extend((x[:8]-prior[:8])*.02)
   return np.asarray(errors)
  opt=least_squares(residual,start,bounds=(lower,upper),max_nfev=150,xtol=1e-10,ftol=1e-10,gtol=1e-10);start=opt.x;q,C=state(start,BR);position=[];normal=[]
  for k,g in enumerate(tips):
   G=v.geom_xmat[g].reshape(3,3);position.append(float(np.linalg.norm(v.geom_xpos[g]+G@Pt[k]-(C+BR@Pc[k]))*1000));normal.append(float(np.linalg.norm(G@Nt[k]+BR@Nc[k])))
  su,sp=inspect(v);qgoal=q+pd;row=dict(theta_deg=float(theta),position_error_mm=position,normal_vector_error=normal,center_translation_mm=((C-C0)*1000).tolist(),max_self_mm=max(sp.values(),default=0.),max_support_mm=max(su.values(),default=0.),evaluations=opt.nfev)
  records.append(row)
  if max(position)>.25 or max(normal)>.03 or row['max_self_mm']>1e-6 or row['max_support_mm']>.15 or np.any(qgoal<clo-1e-10) or np.any(qgoal>chi+1e-10):failed='contact geometry/range residual';break
  knots.append(motor(qgoal))
  if int(theta)%10==0:print(json.dumps(row),flush=True)
 proposal=dict(status='opposed_point_normal_contact_geometry_only',geometry_completed_90=failed is None,failed=failed,knots_completed=len(knots),source_checkpoint=CP.relative_to(R).as_posix(),actual_to_control_offset_source='exact saved forward endpoint, not zero force surface IK',delta_pd_sim_rad=pd.tolist(),native_ranges_unchanged=True,force_gains_and_limits_unchanged=True,
   cube_path_used_only_for_hypothetical_geometry=True,free_body_dynamic_validation_required=True,hardware_execution_allowed=False,devices_opened=False,records=records)
 if failed is None:
  source=np.load(R/'right_hand_rotation_20261030/yaw_roundtrip/candidate.npz');end=int(source['forward_end_sample']);previous=source['goals'][end-1];turn=[]
  for q in knots:
   seg=blend(previous,q);turn.extend(seg);previous=q
  release=blend(previous,source['start']);goals=np.vstack([source['goals'][:end],turn,np.tile(previous,(40,1)),release,np.tile(source['start'],(20,1))]);phases=np.array(source['phases'][:end].tolist()+['stage_7_horizontal_yaw']*len(turn)+['stage_8_horizontal_hold']*40+['stage_9_loaded_open']*len(release)+['stage_10_remove_hold']*20)
  assert np.max(abs(np.diff(np.vstack([source['start'],goals]),axis=0)))<=2 and np.all(goals>=lo) and np.all(goals<=hi)
  np.savez_compressed(O/'candidate.npz',start=source['start'],goals=goals,phases=phases,forward_end_sample=np.array(end));proposal.update(loaded_goals=len(goals),counter_goals=len(turn),nominal_loaded_s=len(goals)*.075,candidate_sha256=hashlib.sha256((O/'candidate.npz').read_bytes()).hexdigest())
 (O/'geometry_review.json').write_text(json.dumps(proposal,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in proposal.items() if k!='records'}),flush=True)
if __name__=='__main__':main()
