"""Free-cube continuations using the previously recorded successful yaw route.

Bridge only the starting posture; preserve native goals,2-tick steps,75ms,
finite actuators and original collision/tracking/retention limits. No devices.
"""
import argparse,json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
R=Path(__file__).resolve().parent;BASE=R/'right_hand_rotation_20261030/natural_grasp_yaw';CP=BASE/'checkpoint'
def blend(a,b):
 n=max(1,int(np.ceil(max(abs(b-a))/2)));return np.rint([a+(b-a)*i/n for i in range(1,n+1)]).astype(int)
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--mode',choices=('direct','linear','early','late'),required=True);args=ap.parse_args();O=BASE/('reference_'+args.mode);O.mkdir(exist_ok=True);source=np.load(R/'right_hand_rotation_20261030/natural_cradle_grasp/candidate.npz');old=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');s=np.load(CP/'grasp_checkpoint.npz');m=mujoco.MjModel.from_binary_path(str(CP/'grasp_checkpoint.mjb'));d=mujoco.MjData(m)
 for k in ('qpos','qvel','ctrl','act','qacc_warmstart'):getattr(d,k)[:]=s[k]
 d.time=float(s['time']);mujoco.mj_forward(m,d);lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1;nlo=(lo[ids]-2048)*TICK;nhi=(hi[ids]-2048)*TICK;up=s['up'];cube=m.geom('cube').id;cb=m.body('cube').id;Binit=s['initial_cube_rotation'];Cinit=s['initial_cube_center'];Bgrasp=d.xmat[cb].reshape(3,3).copy();Cg=d.qpos[-7:-4].copy();newstart=source['goals'][int(source['forward_end_sample'])-1];legacy_start=old['goals'][np.where(old['phases']=='stage_6_grasp_hold')[0][-1]];yaw=old['goals'][old['phases']=='stage_7_horizontal_yaw'];previous=newstart.copy();rows=[];input_bad=None
 for i,base in enumerate(yaw,1):
  t=i/len(yaw);decay=t if args.mode=='linear' else 1-(1-t)**2 if args.mode=='early' else t*t
  target=np.rint(base+(newstart-legacy_start)*(1-decay)).astype(int) if args.mode!='direct' else yaw[-1]
  if np.any(target<lo) or np.any(target>hi):input_bad='reference bridge exceeds native limits; no clipping';break
  part=blend(previous,target);rows.extend(part);previous=target
  if args.mode=='direct':break
 if input_bad:
  (O/'review.json').write_text(json.dumps(dict(status='research_only_rejected',stop=input_bad,hardware_execution_allowed=False,motor_writes=0),indent=2)+'\n',encoding='utf-8');print(input_bad);return
 rows=np.asarray(rows);route=np.vstack([rows,np.tile(previous,(40,1))]);phases=np.array(['stage_7_horizontal_yaw']*len(rows)+['stage_8_horizontal_hold']*40);hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 supports={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,supports);worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);external=False;longest=gap=0.;stop=None;trace=[];peak=None;source_force=m.actuator_forcerange.copy();source_gain=m.actuator_gainprm.copy();source_bias=m.actuator_biasprm.copy()
 def angle(B,base):
  q=Rotation.from_matrix(B@base.T).as_quat();return float(np.degrees(2*np.arctan2(q[:3]@up,q[3])))
 for index,(phase,q) in enumerate(zip(phases,route),1):
  d.ctrl[:]=ticks_to_sim(q)
  for _ in range(75):
   mujoco.mj_step(m,d);su,sp=inspect(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))));inhand=False;contact={}
   for ci,c in enumerate(d.contact):
    a,b=int(c.geom1),int(c.geom2)
    if cube not in (a,b):continue
    other=b if a==cube else a
    if int(m.geom_bodyid[other]) in hand:
     worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000));inhand|=c.dist<.0004;force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);contact[m.geom(other).name]=contact.get(m.geom(other).name,0.)+float(force[0])
    elif other in supports and c.dist<.0004:external=True
   gap=0. if inhand else gap+.001;longest=max(longest,gap)
   if worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25 or np.linalg.norm(d.qpos[-7:-4]-Cinit)>.08:stop='unchanged contact/range/tracking/retention bound';break
  current=d.xmat[cb].reshape(3,3);yaw_value=angle(current,Binit);tilt=float(np.degrees(np.arccos(np.clip((current@np.array([0.,-1,0]))@(Bgrasp@np.array([0.,-1,0])),-1,1))))
  if peak is None or abs(yaw_value-90)<abs(peak['whole_yaw_deg']-90):peak=dict(sample=index,whole_yaw_deg=yaw_value,tilt_deg=tilt)
  if index%20==0 or index==len(route) or stop:
   trace.append(dict(sample=index,phase=str(phase),whole_yaw_deg=yaw_value,delta_from_grasp_deg=angle(current,Bgrasp),tilt_deg=tilt,center_shift_from_grasp_mm=float(np.linalg.norm(d.qpos[-7:-4]-Cg)*1000),contact_N=contact,goal=q.tolist()));print(json.dumps(trace[-1]),flush=True)
  if stop:break
 assert np.array_equal(m.actuator_forcerange,source_force) and np.array_equal(m.actuator_gainprm,source_gain) and np.array_equal(m.actuator_biasprm,source_bias)
 base_n=int(source['forward_end_sample']);allq=np.vstack([source['goals'][:base_n],route]);allph=np.array(source['phases'][:base_n].tolist()+phases.tolist());np.savez_compressed(O/('candidate.npz' if stop is None else 'rejected_candidate.npz'),start=source['start'],goals=allq,phases=allph,forward_end_sample=np.array(len(allq)))
 np.savez_compressed(O/'end_state.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=up,initial_cube_rotation=Binit,initial_cube_center=Cinit)
 report=dict(status='free_cube_natural_grasp_reference_yaw_research',mode=args.mode,safety_passed=stop is None,stop=stop,rotation_rows=len(rows),completed_rows=index,final_whole_yaw_deg=yaw_value,final_delta_from_grasp_deg=angle(current,Bgrasp),peak_toward90=peak,nominal_90_task_met=bool(stop is None and abs(yaw_value-90)<10 and tilt<15),final_tilt_change_deg=tilt,worst=worst,longest_no_hand_contact_s=longest,external_cube_contact=external,trace=trace,native_limits_force_gains_unchanged=True,uses_prior_recorded638_yaw_reference=True,source_checkpoint_sha256=hashlib.sha256((CP/'grasp_checkpoint.npz').read_bytes()).hexdigest(),hardware_execution_allowed=False,motor_writes=0)
 (O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='trace'}),flush=True)
if __name__=='__main__':main()
