"""Compare actual native reach of IF and RF toward a lower Cube side."""
import json
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import differential_evolution,least_squares
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from research_directional_contact_20261030 import support_local,motor
from screen_loaded_repeat_20261026 import cached_contacts
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/lower_three_contact_grasp';CP=O/'initial_checkpoint'
def main():
 m=mujoco.MjModel.from_binary_path(str(CP/'open_cube_checkpoint.mjb'));s=np.load(CP/'open_cube_checkpoint.npz');d=mujoco.MjData(m);d.qpos[:]=s['qpos'];d.qvel[:]=0;mujoco.mj_forward(m,d);cube=m.geom('cube').id;B=d.geom_xmat[cube].reshape(3,3).copy();C=d.geom_xpos[cube].copy();up=s['up']
 ref=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');qref=ticks_to_sim(ref['goals'][np.flatnonzero(ref['phases']=='stage_6_grasp_hold')[-1]]);lo,hi=limits();ids=np.array(SIM_TO_MOTOR_IDS)-1;l=(np.ceil(lo[ids])-2048)*TICK;h=(np.floor(hi[ids])-2048)*TICK
 hand={m.body('palm').id}
 for b in range(m.nbody):
  if int(m.body_parentid[b]) in hand:hand.add(b)
 support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support)
 output=[];horizontal=[k for k in range(3) if abs((B.T@up)[k])<.5]
 for finger,act in (('if',np.arange(4)),('rf',np.arange(8,12))):
  g=m.geom(finger+'_tip').id
  for axis in horizontal:
   for sign in (-1,1):
    n=B[:,axis]*sign;tangent=np.cross(up,n);tangent/=np.linalg.norm(tangent)
    def evaluate(x):
     q=qref.copy();q[act]=x;d.qpos[:]=s['qpos'];d.qpos[:16]=q;d.qvel[:]=0;d.ctrl[:]=q;mujoco.mj_forward(m,d);G=d.geom_xmat[g].reshape(3,3);P=d.geom_xpos[g];p=P+G@support_local(m,g,G.T@n);extent=-up@(G@support_local(m,g,G.T@up));height=up@(P-C);top=height+extent
     su,sp=inspect(d);cp=0.
     for c in d.contact:
      if cube in (int(c.geom1),int(c.geom2)):cp=max(cp,max(0.,-float(c.dist)*1000))
     gap=n@(p-C)-.0275;err=np.array([gap*1000,tangent@(P-C)*1000,(height-min(-.007,.008-extent))*1000]);return err,float(top),max(sp.values(),default=0.),max(su.values(),default=0.),cp,q
    def cost(x):
     e,top,sm,sp,cp,q=evaluate(x);return float(e@e+max(0,sm-.000001)**2*10000+max(0,sp-.15)**2*10000+max(0,cp-.8)**2*10000)
    bounds=list(zip(l[act],h[act]));fit=differential_evolution(cost,bounds,popsize=9,maxiter=120,tol=.0001,polish=True,seed=120+axis+(0 if sign<0 else 5),workers=1);err,top,sm,sp,cp,q=evaluate(fit.x)
    row=dict(finger=finger,axis=axis,sign=sign,cost=float(fit.fun),position_residual_mm=err.tolist(),tip_upper_height_mm=top*1000,max_self_mm=sm,max_support_mm=sp,max_cube_mm=cp,goal_ticks=motor(q).tolist(),geometry_reachable=bool(max(abs(err))<1 and sm<=1e-6 and sp<=.15 and cp<=1 and top<=.010));output.append(row);print(json.dumps(row),flush=True)
 report=dict(status='native_third_finger_reach_comparison_only',cases=output,geometry_reachable_cases=sum(v['geometry_reachable'] for v in output),cube_initial_center=C.tolist(),cube_orientation=B.tolist(),hardware_execution_allowed=False,force_and_geometry_limits_unchanged=True,free_body_validation_required=True,devices_opened=False)
 (O/'third_finger_reach_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
