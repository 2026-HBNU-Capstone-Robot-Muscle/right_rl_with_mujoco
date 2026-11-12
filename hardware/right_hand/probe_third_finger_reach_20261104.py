"""Offline geometric reach candidates for a third-finger support handoff."""
import json
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from propose_natural_grasp_yaw_20261030 import motor
R=Path(__file__).resolve().parent;CP=R/'right_hand_rotation_20261030/natural_grasp_yaw/checkpoint';O=R/'right_hand_rotation_20261104/third_finger_reach'
def main():
 O.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CP/'grasp_checkpoint.mjb'));z=np.load(CP/'grasp_checkpoint.npz');d=mujoco.MjData(m);d.qpos[:]=z['qpos'];d.ctrl[:]=z['ctrl'];d.qvel[:]=0;mujoco.mj_forward(m,d);base=d.qpos.copy();q0=base[:16];pd=z['ctrl']-q0;cube=m.geom('cube').id;B=d.geom_xmat[cube].reshape(3,3).copy();C=d.geom_xpos[cube].copy();mf=m.geom('mf_tip').id;c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,mf}),key=lambda c:c.dist);local=d.geom_xmat[mf].reshape(3,3).T@(c.pos-d.geom_xpos[mf]);normal_local=d.geom_xmat[mf].reshape(3,3).T@(c.frame[:3]*(1 if int(c.geom1)==cube else -1));lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1;low=(np.ceil(lo[ids])+8-2048)*TICK;high=(np.floor(hi[ids])-8-2048)*TICK;hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 support={g for g in range(m.ngeom) if (m.geom(g).name or '').startswith(('support_','leap_mount_collision')) or m.geom(g).name=='floor'};inspect=cached_contacts(m,hand,support);records=[]
 for finger,active in [('rf',np.arange(8,12)),('if',np.arange(0,4))]:
  g=m.geom(finger+'_tip').id;assert int(m.geom_dataid[g])==int(m.geom_dataid[mf]),'Shared CAD contact point requires the same fingertip mesh';lower=np.maximum(m.jnt_range[active,0],low[active]-pd[active]);upper=np.minimum(m.jnt_range[active,1],high[active]-pd[active]);starts=[q0[active]]+[np.array([root,rot,pip,.5]) for root in (.7,1.4) for rot in (-.3,.3) for pip in (.5,1.3)]
  for axis in (0,2):
   for sign in (-1,1):
    for height in (0.,-.005):
     n=B[:,axis]*sign;target=C+n*.0275-B[:,1]*height
     def metrics(x):
      d.qpos[:]=base;d.qpos[active]=x;d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d);G=d.geom_xmat[g].reshape(3,3);error=d.geom_xpos[g]+G@local-target;ne=G@normal_local-n;su,sp=inspect(d);pen=0.
      for c in d.contact:
       if cube in (int(c.geom1),int(c.geom2)):pen=max(pen,max(0.,-float(c.dist)*1000))
      return error,ne,max(sp.values(),default=0.),max(su.values(),default=0.),pen
     best=None
     for initial in starts:
      initial=np.clip(initial,lower+1e-7,upper-1e-7)
      def residual(x):
       e,ne,sm,sp,pen=metrics(x);return np.r_[e*1000,ne,(x-initial)*.01,sm*500,sp*500,max(0.,pen-.8)*100]
      fit=least_squares(residual,initial,bounds=(lower,upper),max_nfev=130,ftol=1e-8,xtol=1e-8,gtol=1e-8);e,ne,sm,sp,pen=metrics(fit.x);err=float(np.linalg.norm(e)*1000);ang=float(np.degrees(np.arccos(np.clip(1-float(ne@ne)/2,-1,1))));good=err<.5 and sm<=1e-6 and sp<=.15 and pen<=1 and ang<45;value=err+sm*1000+sp*1000+max(0,pen-1)*1000;row=dict(finger=finger,cube_face_axis=axis,face_sign=sign,contact_height_mm=height*1000,tip_error_mm=err,normal_error_deg=ang,self_mm=sm,support_mm=sp,cube_mm=pen,geometry_reachable=bool(good),goal=motor(d.ctrl).tolist(),joint_angles=fit.x.tolist(),cost=value)
      if best is None or value<best['cost']:best=row
      if good:break
     records.append(best);print(json.dumps(best),flush=True)
 report=dict(status='third_finger_support_reach_geometry_research',reachable=[r for r in records if r['geometry_reachable']],cases=records,existing_geometries_native_bounds_force_PID_unchanged=True,cube_pinned_for_geometry_only=True,paths_and_free_cube_handoff_not_yet_verified=True,hardware_execution_allowed=False,motor_writes=0);(O/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print('Reachable',len(report['reachable']))
if __name__=='__main__':main()
