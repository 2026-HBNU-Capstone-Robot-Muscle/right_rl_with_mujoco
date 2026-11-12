"""Retreat from the ACTUAL evolved held contact normals, not the old grasp normals."""
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from research_directional_contact_20261030 import motor
R=Path(__file__).resolve().parent;P=R/'right_hand_rotation_20261030/lower_body_gap_release';O=R/'right_hand_rotation_20261030/held_contact_release';CP=R/'right_hand_rotation_20261030/directional_contact_research/checkpoint_nominal'
def blend(a,b):
 n=max(1,int(np.ceil(max(abs(b-a))/2)));return np.rint([a+(b-a)*k/n for k in range(1,n+1)]).astype(int)
def main():
 O.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CP/'forward_checkpoint.mjb'));s=np.load(P/'held_state.npz');d=mujoco.MjData(m);d.qpos[:]=s['qpos'];d.qvel[:]=s['qvel'];d.ctrl[:]=s['ctrl'];mujoco.mj_forward(m,d);cube=m.geom('cube').id;q0=d.qpos[:16].copy();pd=d.ctrl-q0;active=np.array([4,5,6,7,12,13,14,15]);lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1;lower=np.maximum(m.jnt_range[:16,0][active],((np.ceil(lo[ids])-2048)*TICK-pd)[active]);upper=np.minimum(m.jnt_range[:16,1][active],((np.floor(hi[ids])-2048)*TICK-pd)[active]);hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);tips=[m.geom(n).id for n in ('mf_tip','th_tip')];normals=[];points=[];local=[]
 for g in tips:
  contacts=[]
  for ci,c in enumerate(d.contact):
   if {int(c.geom1),int(c.geom2)}=={cube,g}:
    force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);contacts.append((float(force[0]),c))
  force,c=max(contacts,key=lambda v:v[0]);normal=c.frame[:3].copy()*(1 if int(c.geom1)==cube else -1);point=c.pos.copy();normals.append(normal);points.append(point);local.append(d.geom_xmat[g].reshape(3,3).T@(point-d.geom_xpos[g]))
 def state(x):
  q=q0.copy();q[active]=x;d.qpos[:]=s['qpos'];d.qpos[:16]=q;d.qvel[:]=0;d.ctrl[:]=q+pd;mujoco.mj_forward(m,d);return q
 def measure(x,targets):
  q=state(x);errors=np.array([d.geom_xpos[g]+d.geom_xmat[g].reshape(3,3)@lp-target for g,lp,target in zip(tips,local,targets)]);su,sp=inspect(d);cp=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.);return q,errors,max(sp.values(),default=0.),max(su.values(),default=0.),cp
 x=q0[active];records=[];goals=[];previous=motor(s['ctrl']);failed=False
 for gap in np.arange(0.,.0061,.0005):
  targets=[point+n*gap for point,n in zip(points,normals)];before=x.copy()
  def residual(v):
   q,e,sm,su,cp=measure(v,targets);return np.r_[e.ravel()*1000,(v-before)*.06,sm*500,su*500,max(0.,cp-.8)*100]
  opt=least_squares(residual,x,bounds=(lower,upper),max_nfev=150,ftol=1e-9,xtol=1e-9,gtol=1e-9);x=opt.x;q,e,sm,su,cp=measure(x,targets);goal=motor(q+pd);passed=bool(np.max(np.linalg.norm(e,axis=1))*1000<.3 and sm<=1e-6 and su<=.15 and cp<=1 and np.all(goal>=lo) and np.all(goal<=hi));row=dict(gap_mm=gap*1000,point_errors_mm=(np.linalg.norm(e,axis=1)*1000).tolist(),self_mm=sm,support_mm=su,cube_mm=cp,passed=passed,goal=goal.tolist());records.append(row);print(json.dumps(row),flush=True)
  if not passed:failed=True;break
  goals.extend(blend(previous,goal));previous=goal
 meta=dict(status='release_from_evolved_held_normals_geometry',geometry_completed=not failed,records=records,source_held_state_sha256=hashlib.sha256((P/'held_state.npz').read_bytes()).hexdigest(),old_grasp_normals_not_reused=True,force_PID_and_native_bounds_unchanged=True,hardware_execution_allowed=False)
 if not failed:
  z=np.load(P/'candidate.npz');end=int(z['body_hold_end_sample']);values=np.vstack([z['goals'][:end],goals,np.tile(previous,(40,1))]);phases=np.array(z['phases'][:end].tolist()+['stage_15_body_contact_release']*len(goals)+['stage_16_unclamped_remove_hold']*40);assert np.max(abs(np.diff(np.vstack([z['start'],values]),axis=0)))<=2;np.savez_compressed(O/'candidate.npz',start=z['start'],goals=values,phases=phases,forward_end_sample=z['forward_end_sample'],body_hold_end_sample=z['body_hold_end_sample']);meta.update(release_goals=len(goals),candidate_sha256=hashlib.sha256((O/'candidate.npz').read_bytes()).hexdigest())
 (O/'geometry_review.json').write_text(json.dumps(meta,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
