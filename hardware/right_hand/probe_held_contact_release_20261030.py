"""Free-cube release only, beginning from the verified evolved held checkpoint."""
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/held_contact_release';P=R/'right_hand_rotation_20261030/lower_body_gap_release';CP=R/'right_hand_rotation_20261030/directional_contact_research/checkpoint_nominal'
def main():
 m=mujoco.MjModel.from_binary_path(str(CP/'forward_checkpoint.mjb'));s=np.load(P/'held_state.npz');d=mujoco.MjData(m)
 for k in ('qpos','qvel','ctrl','act','qacc_warmstart'):getattr(d,k)[:]=s[k]
 d.time=float(s['time']);mujoco.mj_forward(m,d);z=np.load(O/'candidate.npz');old=np.load(P/'candidate.npz');start=int(z['body_hold_end_sample']);assert np.array_equal(z['goals'][:start],old['goals'][:start]);cube=m.geom('cube').id;body=m.body('cube').id;B=d.xmat[body].reshape(3,3).copy();C=d.qpos[-7:-4].copy();up=-m.opt.gravity/np.linalg.norm(m.opt.gravity);hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 support={g for g in range(m.ngeom) if (m.geom(g).name or '').startswith(('support_','leap_mount_collision')) or m.geom(g).name=='floor'};inspect=cached_contacts(m,hand,support);lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1;low,high=(lo[ids]-2048)*TICK,(hi[ids]-2048)*TICK;worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.,yaw_deg=0.,center_mm=0.);stop=None;rows=[]
 for i in range(start,len(z['goals'])):
  d.ctrl[:]=ticks_to_sim(z['goals'][i])
  for _ in range(75):
   mujoco.mj_step(m,d);su,sp=inspect(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.ctrl-d.qpos[:16]))));worst['range_rad']=max(worst['range_rad'],float(max(np.maximum(low-d.qpos[:16],d.qpos[:16]-high))),0.);v=Rotation.from_matrix(d.xmat[body].reshape(3,3)@B.T).as_quat();yaw=float(np.degrees(2*np.arctan2(v[:3]@up,v[3])));worst['yaw_deg']=max(worst['yaw_deg'],abs(yaw));worst['center_mm']=max(worst['center_mm'],float(np.linalg.norm(d.qpos[-7:-4]-C)*1000))
   for c in d.contact:
    if cube in (int(c.geom1),int(c.geom2)):worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000))
   if worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6:stop='unchanged safety bound';break
  forces={}
  for ci,c in enumerate(d.contact):
   if cube not in (int(c.geom1),int(c.geom2)):continue
   g=int(c.geom2) if int(c.geom1)==cube else int(c.geom1);force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);name=m.geom(g).name;forces[name]=forces.get(name,0.)+float(force[0])
  if (i-start)%10==0 or i==len(z['goals'])-1:rows.append(dict(sample=i+1,yaw_deg=yaw,forces_N=forces))
  if stop:break
 result=dict(status='free_cube_release_from_held_contact_normals',safety_passed=stop is None,stop=stop,worst=worst,rows=rows,final_tip_normals_N={n:forces.get(n,0) for n in ('mf_tip','th_tip')},tips_released=all(forces.get(n,0)<.2 for n in ('mf_tip','th_tip')),candidate_sha256=hashlib.sha256((O/'candidate.npz').read_bytes()).hexdigest(),source_state_sha256=hashlib.sha256((P/'held_state.npz').read_bytes()).hexdigest(),no_cube_pinning_or_pose_reset_during_release=True,hardware_execution_allowed=False,motor_writes=0);result['release_task_passed']=bool(result['safety_passed'] and result['tips_released'] and worst['yaw_deg']<2 and worst['center_mm']<3);(O/'free_release_review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result),flush=True)
if __name__=='__main__':main()
