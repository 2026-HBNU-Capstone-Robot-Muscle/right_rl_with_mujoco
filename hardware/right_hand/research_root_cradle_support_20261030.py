"""Compare small EMPTY root lifts as support before the cube is placed."""
import json
from pathlib import Path
import numpy as np,mujoco
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
R=Path(__file__).resolve().parent;BASE=R/'right_hand_rotation_20261030/stable_support_research';CP=R/'right_hand_rotation_20261030/directional_contact_research/checkpoint_nominal'
def main():
 m=mujoco.MjModel.from_binary_path(str(CP/'forward_checkpoint.mjb'));up=-m.opt.gravity/np.linalg.norm(m.opt.gravity);source=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');seed=np.array(json.loads((BASE/'natural_recess_hypothesis_seed.json').read_bytes())['cube_qpos']);lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1;nlo=(lo[ids]-2048)*TICK;nhi=(hi[ids]-2048)*TICK;cube=m.geom('cube').id;hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);cases=[]
 for offset in (96,160,260):
  folder=BASE/f'root_cradle_{offset}';folder.mkdir(exist_ok=True);goal=source['start'].copy();goal[[7,11]]+=offset;assert np.all(goal>=lo) and np.all(goal<=hi);assert max(goal[[7,11]])<=2472
  d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(source['start']);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d);worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);external=False;stop=None
  def check(with_cube):
   nonlocal external,stop
   su,sp=inspect(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))));normal={}
   if with_cube:
    for ci,c in enumerate(d.contact):
     a,b=int(c.geom1),int(c.geom2)
     if cube not in (a,b):continue
     other=b if a==cube else a
     if int(m.geom_bodyid[other]) in hand:
      worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000));f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);normal[m.geom(other).name]=normal.get(m.geom(other).name,0.)+float(f[0])
     elif other in support and c.dist<.0004:external=True
   if worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external:stop='unchanged_model_safety_limit'
   return normal
  # Collision-free parked cube during EMPTY preparation, then fresh placement.
  for sample in range(1,offset//2+21):
   alpha=min(1.,sample/(offset/2));q=np.rint(source['start']+(goal-source['start'])*alpha).astype(int);d.ctrl[:]=ticks_to_sim(q)
   for _ in range(75):
    mujoco.mj_step(m,d);check(False)
    if stop:break
   if stop:break
  if stop:cases.append(dict(offset_ticks=offset,passed=False,stop=stop,worst=worst));continue
  d.qpos[-7:]=seed;d.qvel[-6:]=0;C=seed[:3].copy();previous=.04;lower=None
  def depth(height):
   d.qpos[-7:-4]=C+up*height;mujoco.mj_forward(m,d);value=0.
   for con in d.contact:
    a,b=int(con.geom1),int(con.geom2)
    if cube in (a,b) and int(m.geom_bodyid[b if a==cube else a]) in hand:value=max(value,max(0.,-float(con.dist)*1000))
   return value
  if depth(previous)>1e-5:raise RuntimeError('Free vertical endpoint intersects hand')
  for height in np.arange(.04,-.0301,-.0005):
   if depth(float(height))>1e-5:lower=float(height);upper=previous;break
   previous=float(height)
  if lower is None:raise RuntimeError('No support in bounded placement search')
  for _ in range(24):
   mid=(lower+upper)/2
   if depth(mid)>1e-5:lower=mid
   else:upper=mid
  d.qpos[-7:-4]=C+up*(upper+.0001);d.qvel[-6:]=0;mujoco.mj_forward(m,d);trace=[]
  for step in range(8001):
   if step:mujoco.mj_step(m,d)
   normal=check(True)
   if step%100==0:trace.append(dict(t=step*.001,center=d.qpos[-7:-4].tolist(),contacts_N=normal,speed=float(np.linalg.norm(d.qvel[-6:-3]))))
   if stop:break
  tail=np.array([v['center'] for v in trace if v['t']>=6.]);span=np.ptp(tail,axis=0)*1000 if len(tail) else np.full(3,np.inf);stable=stop is None and max(span)<1 and float(np.linalg.norm(d.qvel[-6:-3]))<.005 and bool(normal)
  row=dict(offset_ticks=offset,root_lift_deg=offset*TICK*180/np.pi,goal=goal.tolist(),passed=bool(stable),stop=stop,worst=worst,final_center=d.qpos[-7:-4].tolist(),last2s_span_mm=span.tolist(),final_contacts_N=normal,source_cube_stays_free_after_placement=True,hardware_execution_allowed=False);cases.append(row);print(json.dumps(row),flush=True);np.savez_compressed(folder/'state.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=up);(folder/'review.json').write_text(json.dumps(dict(case=row,trace=trace),indent=2)+'\n',encoding='utf-8')
 (BASE/'root_support_comparison.json').write_text(json.dumps(dict(status='empty_root_lift_then_free_cube_support_research',cases=cases,actual_hand_current_PID_native_limits_not_modified=True,hardware_execution_allowed=False,motor_writes=0),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
