"""Device-free trajectory optimization using free-cube simulated feedback.

The cube is never reset in dynamics. A separate kinematic data instance predicts
command-pose contact distance; its pinned geometry is not the task measurement.
Only an offline joint trajectory is produced. Existing actuator/health bounds
and actual CAD geometry remain unchanged. This is not a live SAC policy.
"""
import argparse,json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from propose_natural_grasp_yaw_20261030 import motor
from convex_distance_check_20261104 import CheckedDistance
R=Path(__file__).resolve().parent;BASE=R/'right_hand_rotation_20261030/natural_grasp_yaw';CP=BASE/'checkpoint'
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--press-mm',type=float,choices=(.2,.5),default=.5);ap.add_argument('--source-depth-scale',type=float,choices=(0.,1.,1.5),default=0.);ap.add_argument('--lower-checkpoint',action='store_true');ap.add_argument('--maintain-body-height',action='store_true');args=ap.parse_args();name=f'contact_maintained_{args.press_mm:g}' if not args.source_depth_scale else f'contact_source_depth_{args.source_depth_scale:g}'
 if args.lower_checkpoint:name+='_lower'
 if args.maintain_body_height:name+='_bodyheight'
 BASE_OUT=R/'right_hand_rotation_20261104';BASE_OUT.mkdir(exist_ok=True);O=BASE_OUT/('tangential_'+name);O.mkdir(exist_ok=True);checkpoint=BASE/'lower_checkpoint' if args.lower_checkpoint else CP;source=np.load(BASE/'lower_body_contact/candidate.npz' if args.lower_checkpoint else R/'right_hand_rotation_20261030/natural_cradle_grasp/candidate.npz');prefix_n=int(source['body_hold_end_sample'] if args.lower_checkpoint else source['forward_end_sample']);old=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');s=np.load(checkpoint/'grasp_checkpoint.npz');m=mujoco.MjModel.from_binary_path(str(checkpoint/'grasp_checkpoint.mjb'));d=mujoco.MjData(m);p=mujoco.MjData(m)
 for k in ('qpos','qvel','ctrl','act','qacc_warmstart'):getattr(d,k)[:]=s[k]
 d.time=float(s['time']);mujoco.mj_forward(m,d);lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1;nlo=(lo[ids]-2048)*TICK;nhi=(hi[ids]-2048)*TICK;up=s['up'];cube=m.geom('cube').id;cb=m.body('cube').id;Binit=s['initial_cube_rotation'];Cinit=s['initial_cube_center'];Bgrasp=d.xmat[cb].reshape(3,3).copy();Cg=d.qpos[-7:-4].copy();qstart=source['goals'][prefix_n-1].copy();q= qstart.copy();legacy=old['goals'][old['phases']=='stage_7_horizontal_yaw'][-1];active=np.array([4,5,6,7,12,13,14,15]);tips=[m.geom(n).id for n in ('mf_tip','th_tip')]
 # Extend the swivel only within its existing native travel. Extending all
 # reference joints proportionally would exceed the PIP bound, so that proposal
 # is rejected. Native bounds are not enlarged and no illegal goal is clipped.
 target=legacy.copy();extra_swivel_ticks=max(0,min(256,int(np.floor(hi[9]))-2-int(target[9])));target[9]+=extra_swivel_ticks
 assert np.all(target>=lo) and np.all(target<=hi),'Reference endpoint not native; do not clip.'
 hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 supports={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,supports);worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);external=False;longest=gap=0.;stop=None;trace=[];rows=[];phases=[];goal_seen=False;hold=0;force=m.actuator_forcerange.copy();gain=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();distance_errors=[];closest=None;target_depth=np.full(2,args.press_mm/1000);checked=CheckedDistance(m,cube,tips)
 if args.source_depth_scale:
  start_state=np.load(CP/'grasp_checkpoint.npz');p.qpos[:]=start_state['qpos'];p.qpos[:16]=start_state['ctrl'];p.qvel[:]=0;mujoco.mj_forward(m,p);target_depth=np.array([-float(mujoco.mj_geomDistance(m,p,cube,g,.2,None)) for g in tips])*args.source_depth_scale;assert min(target_depth)>0
 def angle(B,base):
  qq=Rotation.from_matrix(B@base.T).as_quat();return float(np.degrees(2*np.arctan2(qq[:3]@up,qq[3])))
 diagnostic=None;yaw=angle(d.xmat[cb].reshape(3,3),Binit);tilt=0.
 for index in range(1,1401):
  desired=np.rint(qstart+(target-qstart)*min(1.,max(0,index-100)/600)).astype(int)
  if not goal_seen:
   p.qpos[:]=d.qpos;p.qpos[:16]=d.ctrl;p.qvel[:]=0;p.ctrl[:]=d.ctrl;mujoco.mj_forward(m,p);J=[];distances=[];height_jac=[];heights=[];tangent_jac=[];tangent_targets=[];axis=-d.xmat[cb].reshape(3,3)[:,1]
   for g in tips:
    dist,pair=checked.distance(p,g);contacts=[c for c in p.contact if {int(c.geom1),int(c.geom2)}=={cube,g}]
    if contacts:
     c=min(contacts,key=lambda z:z.dist);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1);point=c.pos.copy()
    elif abs(dist)>1e-9:normal=(pair[3:]-pair[:3])/dist;normal/=np.linalg.norm(normal);point=pair[3:]
    else:
     stop='undefined_virtual_contact_normal';diagnostic=dict(geom=m.geom(g).name,geom_distance_m=dist,pair=pair.tolist(),qpos_finite=bool(np.isfinite(d.qpos).all()),probe_finite=bool(np.isfinite(p.qpos).all()),goal=q.tolist(),cube_pose=d.qpos[-7:].tolist());break
    jac=np.zeros((3,m.nv));mujoco.mj_jac(m,p,jac,None,point,int(m.geom_bodyid[g]));J.append(normal@jac[:,active]);distances.append(dist);height_jac.append(axis@jac[:,active]);heights.append(float((point-d.qpos[-7:-4])@axis));project=np.eye(3)-np.outer(normal,normal);tangent_jac.append(project@jac[:,active]);increment=min(.0025,max(0.,90-yaw)*.01745329252);tangent_targets.append(project@np.cross(axis,point-d.qpos[-7:-4])*increment if index>60 else np.zeros(3))
   if stop:break
   J=np.array(J);A=np.vstack([J*10000,np.vstack(tangent_jac)*2000,np.eye(8)*.02]);b=np.r_[(-target_depth-distances)*10000,np.concatenate(tangent_targets)*2000,np.zeros(8)]
   if args.maintain_body_height:A=np.vstack([A,np.array(height_jac)*2000]);b=np.r_[b,(.002-np.array(heights))*2000]
   lower=np.maximum(-2*TICK,nlo[active]-d.ctrl[active]);upper=np.minimum(2*TICK,nhi[active]-d.ctrl[active]);sol=lsq_linear(A,b,bounds=(lower+1e-12,upper-1e-12),tol=1e-10,max_iter=100);control=d.ctrl.copy();control[active]+=sol.x;qnext=motor(control);assert np.max(abs(qnext-q))<=2 and np.all(qnext>=lo) and np.all(qnext<=hi);assert np.array_equal(qnext[[4,5,6,7,12,13,14,15]],qstart[[4,5,6,7,12,13,14,15]]);q=qnext;distance_errors.append([float(v*1000) for v in distances])
  else:hold+=1
  d.ctrl[:]=ticks_to_sim(q);rows.append(q.copy());phases.append('stage_8_horizontal_hold' if goal_seen else 'stage_7_horizontal_yaw')
  for _ in range(75):
   mujoco.mj_step(m,d)
   if not np.isfinite(np.r_[d.qpos,d.qvel,d.ctrl]).all():stop='nonfinite_dynamic_state';break
   su,sp=inspect(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))));inhand=False;contact={}
   for ci,c in enumerate(d.contact):
    a,b=int(c.geom1),int(c.geom2)
    if cube not in (a,b):continue
    other=b if a==cube else a
    if int(m.geom_bodyid[other]) in hand:
     worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000));inhand|=c.dist<.0004;f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);contact[m.geom(other).name]=contact.get(m.geom(other).name,0.)+float(f[0])
    elif other in supports and c.dist<.0004:external=True
   gap=0. if inhand else gap+.001;longest=max(longest,gap)
   if worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25 or np.linalg.norm(d.qpos[-7:-4]-Cinit)>.08:stop='unchanged contact/range/tracking/retention bound';break
  B=d.xmat[cb].reshape(3,3);yaw=angle(B,Binit);tilt=float(np.degrees(np.arccos(np.clip((B@np.array([0.,-1,0]))@(Bgrasp@np.array([0.,-1,0])),-1,1))))
  if closest is None or abs(yaw-90)<abs(closest['whole_yaw_deg']-90):closest=dict(sample=index,whole_yaw_deg=yaw,tilt_deg=tilt)
  if not goal_seen and yaw>=88 and tilt<15:goal_seen=True
  if index%40==0 or stop or goal_seen and hold in (0,67):
   row=dict(sample=index,whole_yaw_deg=yaw,tilt_deg=tilt,goal=q.tolist(),contact_N=contact,command_pose_tip_distances_mm=distance_errors[-1],tracking_rad=float(max(abs(d.qpos[:16]-d.ctrl))));trace.append(row);print(json.dumps(row),flush=True)
  if stop or goal_seen and hold>=67:break
 assert np.array_equal(m.actuator_forcerange,force) and np.array_equal(m.actuator_gainprm,gain) and np.array_equal(m.actuator_biasprm,bias)
 n=prefix_n;allq=np.vstack([source['goals'][:n],rows]);allph=np.array(source['phases'][:n].tolist()+phases);np.savez_compressed(O/('candidate.npz' if stop is None else 'rejected_candidate.npz'),start=source['start'],goals=allq,phases=allph,forward_end_sample=np.array(len(allq)));np.savez_compressed(O/'end_state.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=up,initial_cube_rotation=Binit,initial_cube_center=Cinit)
 report=dict(status='offline_free_cube_tangential_contact_yaw_research',safety_passed=stop is None,stop=stop,diagnostic=diagnostic,corrected_geometry_distance_calls=len(checked.corrections),rows=len(rows),extra_swivel_ticks=extra_swivel_ticks,virtual_command_contact_depth_mm=(target_depth*1000).tolist(),lower_checkpoint=args.lower_checkpoint,maintain_body_contact_height=args.maintain_body_height,whole_yaw_deg=yaw,closest_to90=closest,tilt_deg=tilt,task90_met=bool(stop is None and goal_seen and hold>=67 and abs(yaw-90)<10),worst=worst,trace=trace,simulation_cube_pose_never_reset=True,separate_geometry_probe_is_not_dynamic_task_evidence=True,live_SAC=False,native_force_gains_safety_unchanged=True,independent_positive_distance_check='convex_distance_validation.json',source_checkpoint_sha256=hashlib.sha256((checkpoint/'grasp_checkpoint.npz').read_bytes()).hexdigest(),hardware_execution_allowed=False,motor_writes=0)
 (O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='trace'}),flush=True)
if __name__=='__main__':main()
