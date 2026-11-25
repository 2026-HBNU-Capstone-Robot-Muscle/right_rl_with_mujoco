"""Free-body support comparison for the actual hands-free recess placement.

Marker-to-palm cardinal axes are an unverified modeling hypothesis, never a
camera-to-motor controller. No geometry, forces, gains, or joint limits change.
"""
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/stable_support_research';CP=R/'right_hand_rotation_20261030/directional_contact_research/checkpoint_nominal'
def main():
 observed=json.loads((O/'natural_placement_measurement.json').read_bytes());ref=json.loads((R/'right_hand_rotation_20261030/placement_alignment_capture/image_reference.json').read_bytes());base=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1;nlo=(lo[ids]-2048)*TICK;nhi=(hi[ids]-2048)*TICK
 m=mujoco.MjModel.from_binary_path(str(CP/'forward_checkpoint.mjb'));assert m.opt.timestep==.001;cube=m.geom('cube').id;cb=m.body('cube').id;palm=m.body('palm').id;up=-m.opt.gravity/np.linalg.norm(m.opt.gravity);hand={palm}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support)
 prior=json.loads((R/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());seed=np.array(next(v['settled_cube_qpos'] for v in prior['loaded_physics'] if v['book_front_mm']==-10 and v['gain_scale']==1 and v['friction']==.3));d0=mujoco.MjData(m);d0.qpos[:16]=ticks_to_sim(base['start']);d0.ctrl[:]=d0.qpos[:16];d0.qpos[-7:]=seed;mujoco.mj_forward(m,d0)
 marker_to_palm=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]]);marker_to_world=d0.xmat[palm].reshape(3,3)@marker_to_palm;assert np.linalg.det(marker_to_world)>.999
 delta=np.array(observed['delta_from_old_reference_median_mm'])/1000;delta_world=marker_to_world@delta;orientation=marker_to_world@np.array(observed['orange_rotation_body'])@np.array(ref['orange_rotation_body']).T@marker_to_world.T
 seedR=Rotation.from_quat(np.r_[seed[4:7],seed[3]]).as_matrix();newR=orientation@seedR;quat=Rotation.from_matrix(newR).as_quat();natural=seed.copy();natural[:3]+=delta_world;natural[3:]=np.r_[quat[3],quat[:3]];reports=[]
 for name,pose in [('previous_reference_seed',seed),('natural_recess_hypothesis',natural)]:
  d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(base['start']);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=pose;d.qvel[:]=0;mujoco.mj_forward(m,d);center=pose[:3].copy();previous=.04;lower=None
  def depth(height):
   d.qpos[-7:-4]=center+up*height;mujoco.mj_forward(m,d);value=0.
   for c in d.contact:
    a,b=int(c.geom1),int(c.geom2)
    if cube in (a,b) and int(m.geom_bodyid[b if a==cube else a]) in hand:value=max(value,max(0.,-float(c.dist)*1000))
   return value
  if depth(previous)>1e-5:raise RuntimeError('Upper free endpoint intersects hand')
  for h in np.arange(.04,-.0301,-.0005):
   if depth(float(h))>1e-5:lower=float(h);upper=previous;break
   previous=float(h)
  if lower is None:raise RuntimeError('No support found on bounded seating line')
  for _ in range(24):
   mid=(lower+upper)/2
   if depth(mid)>1e-5:lower=mid
   else:upper=mid
  shift=upper+.0001;d.qpos[-7:-4]=center+up*shift;d.qvel[:]=0;mujoco.mj_forward(m,d);worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);external=False;trace=[];stop=None
  for step in range(8001):
   if step:mujoco.mj_step(m,d)
   su,sp=inspect(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0.,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))));contacts={}
   for ci,c in enumerate(d.contact):
    a,b=int(c.geom1),int(c.geom2)
    if cube not in (a,b):continue
    other=b if a==cube else a
    if int(m.geom_bodyid[other]) in hand:
     worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000));f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);contacts[m.geom(other).name]=contacts.get(m.geom(other).name,0.)+float(f[0])
    elif other in support and c.dist<.0004:external=True
   if step%100==0:trace.append(dict(t=step*.001,center=d.qpos[-7:-4].tolist(),quaternion=d.qpos[-4:].tolist(),contacts_N=contacts,speed_m_s=float(np.linalg.norm(d.qvel[-6:-3]))))
   if worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external:stop='original_contact_range_tracking_or_external_support';break
  tail=[r['center'] for r in trace if r['t']>=6.];span=np.ptp(tail,axis=0)*1000 if tail else np.full(3,np.inf);stable=stop is None and len(tail)>=10 and max(span)<1. and float(np.linalg.norm(d.qvel[-6:-3]))<.005 and bool(contacts)
  result=dict(name=name,passed=bool(stable),stop=stop,upper_seating_endpoint_free=True,seating_shift_mm=float(shift*1000),seed_center=pose[:3].tolist(),settled_center=d.qpos[-7:-4].tolist(),last2s_center_span_mm=span.tolist(),final_contacts_N=contacts,worst=worst,trace=trace);reports.append(result);print(json.dumps({k:v for k,v in result.items() if k!='trace'}),flush=True)
  np.savez_compressed(O/(name+'_state.npz'),qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=up)
  (O/(name+'_seed.json')).write_text(json.dumps(dict(cube_qpos=pose.tolist(),unverified_cardinal_registration_hypothesis=True,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
 report=dict(status='support_stability_model_research_only',marker_to_world=marker_to_world.tolist(),observed_delta_body_mm=(delta*1000).tolist(),hypothesized_delta_world_mm=(delta_world*1000).tolist(),cardinal_axes_hypothesis_not_3D_registration=True,force_PID_native_geometry_unchanged=True,source_observation_sha256=hashlib.sha256((O/'natural_placement_measurement.json').read_bytes()).hexdigest(),cases=reports,hardware_execution_allowed=False,motor_writes=0)
 (O/'support_stability_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
