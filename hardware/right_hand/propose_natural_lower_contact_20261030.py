"""Release, lower thumb contact, reclose below the upper layer. Geometry only."""
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from research_directional_contact_20261030 import motor
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/natural_grasp_yaw/lower_body_contact';CP=R/'right_hand_rotation_20261030/natural_grasp_yaw/checkpoint'
def blend(a,b):
 n=max(1,int(np.ceil(max(abs(b-a))/2)));return np.rint([a+(b-a)*k/n for k in range(1,n+1)]).astype(int)
def upper_gap_factory(m,g,B,C):
 center=C+B@np.array([0.,-.055/3,0.]);size=np.array([.0275,.055/6,.0275]);kind=int(m.geom_type[g])
 if kind==7:
  mid=int(m.geom_dataid[g]);a=int(m.mesh_vertadr[mid]);verts=np.asarray(m.mesh_vert[a:a+int(m.mesh_vertnum[mid])],float);hull=ConvexHull(verts);points=verts[hull.vertices];normals=hull.equations[:,:3];edges=set()
  for tri in hull.simplices:
   for x,y in ((tri[0],tri[1]),(tri[1],tri[2]),(tri[2],tri[0])):edges.add(tuple(sorted((int(x),int(y)))))
  directions=np.array([verts[y]-verts[x] for x,y in edges])
 elif kind==6:
  points=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*m.geom_size[g];normals=np.eye(3);directions=np.eye(3)
 else:raise RuntimeError('Unsupported upper-layer clearance geometry')
 def unique(a):
  length=np.linalg.norm(a,axis=1);a=a[length>1e-9]/length[length>1e-9,None];return np.unique(np.round(a,8),axis=0)
 normals=unique(normals);directions=unique(directions)
 def gap(d):
  G=d.geom_xmat[g].reshape(3,3);nn=normals@G.T;ee=directions@G.T;cross=np.cross(B.T[:,None,:],ee[None,:,:]).reshape(-1,3);length=np.linalg.norm(cross,axis=1);cross=cross[length>1e-9]/length[length>1e-9,None];axes=np.vstack([B.T,nn,cross]);rad=np.abs(axes@B)@size;v=points@G.T+(d.geom_xpos[g]-center);proj=v@axes.T;return float(np.max(np.maximum(proj.min(0)-rad,-rad-proj.max(0))))
 return gap
def main():
 O.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CP/'grasp_checkpoint.mjb'));s=np.load(CP/'grasp_checkpoint.npz');d=mujoco.MjData(m);d.qpos[:]=s['qpos'];d.qvel[:]=0;d.ctrl[:]=s['ctrl'];mujoco.mj_forward(m,d);q0=d.qpos[:16].copy();pd=s['ctrl']-q0;up=s['up'];cube=m.geom('cube').id;B=d.geom_xmat[cube].reshape(3,3).copy();C=d.geom_xpos[cube].copy();tips=[m.geom(n).id for n in ('mf_tip','th_tip')];points=[];normals=[];local=[]
 for g in tips:
  cc=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist);p=cc.pos.copy();n=cc.frame[:3].copy()*(1 if int(cc.geom1)==cube else -1);points.append(p);normals.append(n);local.append(d.geom_xmat[g].reshape(3,3).T@(p-d.geom_xpos[g]))
 hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 supports={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,supports);active=np.array([4,5,6,7,12,13,14,15]);lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1;ql=(np.ceil(lo[ids])-2048)*TICK;qh=(np.floor(hi[ids])-2048)*TICK;lower=np.maximum(m.jnt_range[:16,0][active],ql[active]-pd[active]);upper=np.minimum(m.jnt_range[:16,1][active],qh[active]-pd[active])
 thumb_geoms=[g for g in range(m.ngeom) if m.body(int(m.geom_bodyid[g])).name.startswith(('th_','mf_')) and (m.geom_contype[g] or m.geom_conaffinity[g])];upper_gaps={g:upper_gap_factory(m,g,B,C) for g in thumb_geoms if int(m.geom_type[g]) in (6,7)}
 def state(x):
  q=q0.copy();q[active]=x;d.qpos[:]=s['qpos'];d.qpos[:16]=q;d.qvel[:]=0;d.ctrl[:]=q+pd;mujoco.mj_forward(m,d);return q
 def metrics(x,targets):
  q=state(x);errors=[d.geom_xpos[g]+d.geom_xmat[g].reshape(3,3)@lp-p for g,lp,p in zip(tips,local,targets)];su,sp=inspect(d);cp=0.
  for c in d.contact:
   if cube in (int(c.geom1),int(c.geom2)):cp=max(cp,max(0.,-float(c.dist)*1000))
  ug=min((f(d) for f in upper_gaps.values()),default=1.);return errors,max(sp.values(),default=0.),max(su.values(),default=0.),cp,ug,q
 current=q0[active];knots=[];records=[];failed=None;stages=[('initial',0.,0.,None)]
 stages += [('release',g,0.,None) for g in (.0005,.001,.0015,.002,.0025,.003)]
 stages += [('lower_thumb',.003,h,.0005) for h in np.arange(.002,.0161,.002)]
 stages += [('reclose',g,.016,.0005) for g in (.0025,.002,.0015,.001,.0005,0.)]
 for phase,gap,drop,upper_required in stages:
  targets=[points[0]+normals[0]*gap-up*(drop/4),points[1]+normals[1]*gap-up*drop];previous=current.copy()
  def residual(x):
   errors,sm,sp,cp,ug,q=metrics(x,targets);r=np.r_[np.array(errors).ravel()*1000,(x-previous)*.05,sm*500,sp*500,max(0.,cp-.8)*80]
   if upper_required is not None:r=np.r_[r,max(0.,upper_required-ug)*10000]
   return r
  opt=least_squares(residual,current,bounds=(lower,upper),max_nfev=180,ftol=1e-9,xtol=1e-9,gtol=1e-9);current=opt.x;errors,sm,sp,cp,ug,q=metrics(current,targets);position=[float(np.linalg.norm(e)*1000) for e in errors];goal=motor(q+pd)
  good=bool(max(position)<.3 and sm<=1e-6 and sp<=.15 and cp<=1 and np.all(goal>=lo) and np.all(goal<=hi) and (upper_required is None or ug>=upper_required-1e-5))
  row=dict(phase=phase,gap_mm=gap*1000,thumb_lowering_mm=drop*1000,tip_point_errors_mm=position,max_self_mm=sm,max_support_mm=sp,max_cube_mm=cp,min_MF_TH_upper_layer_gap_mm=ug*1000,passed_geometry=good,goal=goal.tolist(),evaluations=opt.nfev);records.append(row);print(json.dumps(row),flush=True)
  if not good:failed='contact/range/upper-layer clearance geometry';break
  knots.append((phase,goal))
 report=dict(status='post_natural_grasp_lower_body_contact_geometry_only',geometry_completed=failed is None,failed=failed,source_checkpoint=CP.relative_to(R).as_posix(),upper_layer_box_is_top18_33mm_of_existing55mm_cube=True,upper_layer_clearance_not_a_complete_left_tool_clearance_certificate=True,
  model_thumb_contact_height_before_mm=float((points[1]-C)@(B@np.array([0.,-1.,0]))*1000),both_fingers_release_then_thumb_and_middle_lower=True,middle_point_lowering_mm=4.0,source_cube_pinned_only_for_design_not_dynamic_validation=True,force_PID_and_native_limits_unchanged=True,hardware_execution_allowed=False,records=records,devices_opened=False)
 if failed is None:
  src=np.load(R/'right_hand_rotation_20261030/natural_cradle_grasp/candidate.npz');end=int(src['forward_end_sample']);previous=src['goals'][end-1];goals=[];phases=[]
  for phase,q in knots:
   v=blend(previous,q);goals.extend(v);phases.extend([{'initial':'stage_11_body_release','release':'stage_11_body_release','lower_thumb':'stage_12_body_reseat','reclose':'stage_13_body_close'}[phase]]*len(v));previous=q
  post=np.asarray(goals);release=blend(previous,src['start']);allq=np.vstack([src['goals'][:end],post,np.tile(previous,(107,1)),release,np.tile(src['start'],(20,1))]);allp=np.array(src['phases'][:end].tolist()+phases+['stage_14_body_hold']*107+['stage_9_loaded_open']*len(release)+['stage_10_remove_hold']*20)
  assert np.max(abs(np.diff(np.vstack([src['start'],allq]),axis=0)))<=2;np.savez_compressed(O/'candidate.npz',start=src['start'],goals=allq,phases=allp,forward_end_sample=np.array(end),body_hold_end_sample=np.array(end+len(post)+107));report.update(post_goals=len(post),loaded_goals=len(allq),nominal_loaded_s=len(allq)*.075,candidate_sha256=hashlib.sha256((O/'candidate.npz').read_bytes()).hexdigest())
 (O/'geometry_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
