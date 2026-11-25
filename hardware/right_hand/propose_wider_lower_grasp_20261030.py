"""Design a lower three-finger holding pose; no devices or force/PID changes."""
import json,math,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from research_directional_contact_20261030 import support_local,motor
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/lower_three_contact_grasp/wider_center';CP=R/'right_hand_rotation_20261030/lower_three_contact_grasp/initial_checkpoint'
def main():
 O.mkdir(parents=True,exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CP/'open_cube_checkpoint.mjb'));s=np.load(CP/'open_cube_checkpoint.npz');d=mujoco.MjData(m);d.qpos[:]=s['qpos'];d.qvel[:]=0;d.ctrl[:]=s['ctrl'];mujoco.mj_forward(m,d)
 up=s['up'];cube=m.geom('cube').id;cb=m.body('cube').id;C0=d.xpos[cb].copy();B=d.xmat[cb].reshape(3,3).copy();tips=[m.geom(n).id for n in ('if_tip','mf_tip','th_tip')];active=np.r_[np.arange(8),np.arange(12,16)]
 src=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');grip=src['goals'][np.flatnonzero(src['phases']=='stage_6_grasp_hold')[-1]];qref=ticks_to_sim(grip);qref[:4]=qref[4:8];qref[1]=-.35;qref[0]=.4
 lo,hi=limits();sid=np.array(SIM_TO_MOTOR_IDS)-1;nlo=(np.ceil(lo[sid])-2048)*TICK;nhi=(np.floor(hi[sid])-2048)*TICK
 e0=np.array([1.,0,0]);e0-=up*(e0@up);e0/=np.linalg.norm(e0);basis=np.column_stack([e0,np.cross(up,e0),up]);hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support)
 def state(x):
  q=ticks_to_sim(grip);q[active]=x[:12];C=C0+basis@x[12:];d.qpos[:]=s['qpos'];d.qpos[:16]=q;d.qpos[-7:-4]=C;d.qvel[:]=0;d.ctrl[:]=q;mujoco.mj_forward(m,d);return q,C
 state(np.r_[np.clip(qref[active],nlo[active],nhi[active]),[0.,0.,0.]])
 horizontal=[i for i in range(3) if abs((B.T@up)[i])<.5];mfdelta=B.T@(d.geom_xpos[tips[1]]-C0);mfaxis=max(horizontal,key=lambda k:abs(mfdelta[k]));other=next(k for k in horizontal if k!=mfaxis);sgn=np.sign(mfdelta[mfaxis]) or 1.
 normals=[]
 for axis,sign in ((other,np.sign((B.T@(d.geom_xpos[tips[0]]-C0))[other]) or 1.),(mfaxis,sgn),(mfaxis,-sgn)):
  a=np.zeros(3);a[axis]=sign;normals.append(B@a)
 localnorms=[]
 for g,n in zip(tips,normals):
  k=int(np.argmin(m.geom_size[g]));a=np.zeros(3);a[k]=np.sign(d.geom_xmat[g].reshape(3,3)[:,k]@(-n)) or 1.;localnorms.append(a)
 lower=np.r_[nlo[active],[-.03,-.03,0.]];upper=np.r_[nhi[active],[.03,.03,.003]];candidates=[];best=None
 def metrics(x):
  q,C=state(x);rows=[]
  for k,(g,n) in enumerate(zip(tips,normals)):
   G=d.geom_xmat[g].reshape(3,3);P=d.geom_xpos[g];toward=P+G@support_local(m,g,G.T@n);extentup=float(-up@(G@support_local(m,g,G.T@up)))
   tangent=np.cross(up,n);tangent/=np.linalg.norm(tangent);gap=float(n@(toward-C)-.0275);height=float(up@(P-C));upper_height=height+extentup
   rows.append(dict(gap_m=gap,tangent_m=float(tangent@(P-C)),center_height_m=height,top_height_m=upper_height,normal_residual=(G@localnorms[k]+n).tolist()))
  su,sp=inspect(d);cube_pen=0.
  for c in d.contact:
   if cube in (int(c.geom1),int(c.geom2)):cube_pen=max(cube_pen,max(0.,-float(c.dist)*1000))
  return rows,max(sp.values(),default=0.),max(su.values(),default=0.),cube_pen
 def residual(x):
  rows,selfmm,supmm,cp=metrics(x);out=[]
  for row in rows:
   desired_height=min(-.007,.008-(row['top_height_m']-row['center_height_m']))
   out.extend([(row['gap_m']+.0004)*1000,row['tangent_m']*150,(row['center_height_m']-desired_height)*1000]);out.extend(np.array(row['normal_residual'])*0)
  out.extend([selfmm*500,supmm*500,max(0.,cp-.8)*80]);out.extend(x[12:]*50);out.extend((x[:12]-qref[active])*.02);return np.array(out)
 for idx,(rot,flex) in enumerate(((-.7,.5),(-.35,.5),(0.,.5),(.35,.5),(-.35,1.0),(0.,1.0),(.35,1.0))):
  seed=qref.copy();seed[1]=rot;seed[0]=flex;x=np.r_[np.clip(seed[active],lower[:12]+1e-6,upper[:12]-1e-6),[0.,0.,.0005]]
  opt=least_squares(residual,x,bounds=(lower,upper),max_nfev=300,ftol=1e-9,xtol=1e-9,gtol=1e-9);rows,sm,sp,cp=metrics(opt.x);q,C=state(opt.x)
  passed=bool(max(abs(v['gap_m']) for v in rows)<.0008 and all(v['top_height_m']<.010 for v in rows) and sm<=1e-6 and sp<=.15 and cp<=1.)
  row=dict(seed=idx,passed_geometry=passed,cost=float(np.linalg.norm(opt.fun)),evaluations=opt.nfev,max_self_mm=sm,max_support_mm=sp,max_cube_mm=cp,tips=rows,goal_ticks=motor(q).tolist(),cube_center=C.tolist(),center_shift_mm=((C-C0)*1000).tolist());candidates.append(row);print(json.dumps(row),flush=True)
  if passed and (best is None or row['cost']<best['cost']):best=row
 report=dict(status='three_contact_lower_grasp_geometry_research',goal='Stable lower body grasp with IF/MF/TH; upper layer clearance is a design objective.',candidates=candidates,selected=best,geometry_candidate_found=best is not None,
  native_ranges_unchanged=True,force_PID_not_increased=True,virtual_cube_used_only_in_geometry_design=True,contact_normal_alignment_is_soft_design_preference_disabled=True,center_search_planar_mm=30,free_body_approach_and_disturbance_tests_required=True,real_current_force_not_calibrated=True,physical_grasp_not_verified=True,hardware_execution_allowed=False,devices_opened=False)
 if best is not None:np.savez_compressed(O/'geometry_candidate.npz',start=src['start'],goal=np.array(best['goal_ticks']),cube_center=np.array(best['cube_center']),cube_rotation=B,up=up)
 (O/'geometry_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print('GEOMETRY CANDIDATE',best is not None,flush=True)
if __name__=='__main__':main()
