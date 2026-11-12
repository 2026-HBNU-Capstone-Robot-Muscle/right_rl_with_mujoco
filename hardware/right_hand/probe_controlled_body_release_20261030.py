"""Free-body continuation from the exact completed forward checkpoint. No devices."""
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from propose_lower_thumb_regrasp_20261030 import upper_gap_factory
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/lower_body_gap_release';CP=R/'right_hand_rotation_20261030/directional_contact_research/checkpoint_nominal'
def main():
 m=mujoco.MjModel.from_binary_path(str(CP/'forward_checkpoint.mjb'));s=np.load(CP/'forward_checkpoint.npz');z=np.load(O/'candidate.npz');source=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');end=int(z['forward_end_sample']);assert np.array_equal(z['goals'][:end],source['goals'][:end]);assert abs(m.opt.timestep-.001)<1e-12;d=mujoco.MjData(m)
 def restore(saved):
  for k,v in saved.items():
   if k=='time':d.time=float(v)
   else:getattr(d,k)[:]=v
  d.xfrc_applied[:]=0;mujoco.mj_forward(m,d)
 saved0={k:s[k] for k in ('qpos','qvel','ctrl','act','qacc_warmstart','time')};restore(saved0);up=s['up'];body=m.body('cube').id;cube=m.geom('cube').id;hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 supports={g for g in range(m.ngeom) if (m.geom(g).name or '').startswith(('support_','leap_mount_collision')) or m.geom(g).name=='floor'};inspect=cached_contacts(m,hand,supports);lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1;lower=(lo[ids]-2048)*TICK;upper=(hi[ids]-2048)*TICK
 def rotation():return d.xmat[body].reshape(3,3).copy()
 def angle(B,A):
  quat=Rotation.from_matrix(B@A.T).as_quat();return float(np.degrees(2*np.arctan2(quat[:3]@up,quat[3])))
 def snapshot():return {**{k:getattr(d,k).copy() for k in ('qpos','qvel','ctrl','act','qacc_warmstart')},'time':d.time}
 B0=rotation();C0=d.qpos[-7:-4].copy();worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.,yaw_drift_deg=0.,center_shift_mm=0.,longest_no_hand_s=0.);stop=None;nohand=0.;samples=[];held=None;probes=[]
 def inspect_step():
  nonlocal nohand,stop
  su,sp=inspect(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(np.maximum(lower-d.qpos[:16],d.qpos[:16]-upper))),0.);inhand=False;external=False
  for c in d.contact:
   a,b=int(c.geom1),int(c.geom2)
   if cube not in (a,b):continue
   other=b if a==cube else a
   if int(m.geom_bodyid[other]) in hand:worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000));inhand|=c.dist<.0004
   elif other in supports and c.dist<.0004:external=True
  nohand=0. if inhand else nohand+.001;worst['longest_no_hand_s']=max(worst['longest_no_hand_s'],nohand);worst['yaw_drift_deg']=max(worst['yaw_drift_deg'],abs(angle(rotation(),B0)));worst['center_shift_mm']=max(worst['center_shift_mm'],float(np.linalg.norm(d.qpos[-7:-4]-C0)*1000))
  if worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or nohand>.25:stop='unchanged self/support/cube/tracking/range/retention bound'
 def contact_report():
  B=rotation();C=d.qpos[-7:-4].copy();heights={};normal={}
  for ci,c in enumerate(d.contact):
   if cube not in (int(c.geom1),int(c.geom2)):continue
   g=int(c.geom2) if int(c.geom1)==cube else int(c.geom1);name=m.geom(g).name
   if int(m.geom_bodyid[g]) not in hand:continue
   force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);normal[name]=normal.get(name,0.)+float(force[0]);heights.setdefault(name,[]).append(float((c.pos-C)@(B@np.array([0.,-1.,0]))*1000))
  gaps={}
  for g in range(m.ngeom):
   name=m.body(int(m.geom_bodyid[g])).name
   if name.startswith(('th_','mf_')) and (m.geom_contype[g] or m.geom_conaffinity[g]) and int(m.geom_type[g]) in (6,7):gaps[m.geom(g).name]=upper_gap_factory(m,g,B,C)(d)*1000
  return dict(contact_heights_mm=heights,normal_forces_N=normal,min_MF_TH_upper_layer_gap_mm=min(gaps.values()),upper_layer_gap_by_geom_mm=gaps)
 new_hold_end=int(z['body_hold_end_sample'])
 for index in range(end,len(z['goals'])):
  d.ctrl[:]=ticks_to_sim(z['goals'][index])
  for _ in range(75):
   mujoco.mj_step(m,d);inspect_step()
   if stop:break
  if index%20==0 or index+1==new_hold_end or stop:samples.append(dict(sample=index+1,phase=str(z['phases'][index]),yaw_change_deg=angle(rotation(),B0),center_change_mm=float(np.linalg.norm(d.qpos[-7:-4]-C0)*1000),**contact_report()))
  if stop:break
  if index+1==new_hold_end:
   held=snapshot();np.savez_compressed(O/'held_state.npz',**held);heldR=rotation();heldC=d.qpos[-7:-4].copy();print('HELD',json.dumps(contact_report()),flush=True)
   for torque in ():
    restore(held);drift=shift=0.;counts={'mf_tip':0,'th_tip':0};heights={'mf_tip':[],'th_tip':[]};sm=su=cp=tr=0.;probe_stop=None
    for k in range(3000):
     d.xfrc_applied[body,3:]=up*(torque if k<1000 else 0.);mujoco.mj_step(m,d);p_sup,p_self=inspect(d);sm=max(sm,max(p_self.values(),default=0.));su=max(su,max(p_sup.values(),default=0.));tr=max(tr,float(max(abs(d.qpos[:16]-d.ctrl))));drift=max(drift,abs(angle(rotation(),heldR)));shift=max(shift,float(np.linalg.norm(d.qpos[-7:-4]-heldC)*1000));normal={}
     for ci,c in enumerate(d.contact):
      if cube not in (int(c.geom1),int(c.geom2)):continue
      g=int(c.geom2) if int(c.geom1)==cube else int(c.geom1);name=m.geom(g).name
      if int(m.geom_bodyid[g]) in hand:cp=max(cp,max(0.,-float(c.dist)*1000))
      if name in counts:
       force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);normal[name]=normal.get(name,0.)+float(force[0]);heights[name].append(float((c.pos-d.qpos[-7:-4])@(rotation()@np.array([0.,-1.,0]))*1000))
     for name in counts:counts[name]+=normal.get(name,0.)>.2
     if sm>1e-6 or su>.15 or cp>1 or tr>.18:probe_stop='unchanged geometry/tracking';break
    report=dict(torque_Nm=torque,design_torque_not_measured_layer_load=True,max_yaw_drift_deg=drift,max_center_shift_mm=shift,max_self_mm=sm,max_support_mm=su,max_cube_mm=cp,max_tracking_rad=tr,stop=probe_stop,opposing_contact_fraction={name:n/(k+1) for name,n in counts.items()},tip_heights_mm={name:[min(v),max(v)] if v else None for name,v in heights.items()});report['body_hold_task_passed']=bool(probe_stop is None and drift<5 and shift<3 and all(n/(k+1)>.8 for n in counts.values()) and all(v and max(v)<.055/6*1000 for v in heights.values()));probes.append(report);print('PROBE',json.dumps(report),flush=True)
   restore(held)
 report=dict(status='lower_body_regrasp_free_cube_continuation',stop=stop,last_sample=index+1,goals=len(z['goals']),safety_route_passed=stop is None,worst=worst,samples=samples,hold_probes=probes,body_hold_task_all_probes_passed=None,torque_probes_not_repeated_here=True,identical_held_prefix_torque_probes='right_hand_rotation_20261030/lower_body_regrasp/free_continuation_review.json',source_prefix_exact_match=True,checkpoint=CP.relative_to(R).as_posix(),checkpoint_geometry_and_motion_state_preserved=True,no_cube_reset_or_pinning_within_each_trial=True,separate_torque_probe_states_restored_explicitly=True,candidate_sha256=hashlib.sha256((O/'candidate.npz').read_bytes()).hexdigest(),native_force_PID_bounds_unchanged=True,controlled_contact_release_no_full_OPEN=True,final_released_state=contact_report(),physical_rotation_and_hold_not_verified=True,hardware_execution_allowed=False,devices_opened=False)
 (O/'free_continuation_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print('RESULT',report['safety_route_passed'],report['body_hold_task_all_probes_passed'],flush=True)
if __name__=='__main__':main()
