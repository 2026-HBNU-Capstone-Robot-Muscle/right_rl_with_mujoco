"""Geometric search for an additional lower-body contact after a turn.

Both face contact coordinates vary rather than assuming the face centre is
reachable. Endpoint geometry is a proposal, not free-cube or hardware evidence.
"""
import json,time
import numpy as np,mujoco
from scipy.optimize import least_squares
from research_rotation_body_hold_20261104 import CP,OUT,restore,motor,native,inspect_for,evaluate
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK

def main():
    m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));z=np.load(OUT/'turn_0_lower_8_normal_10/end_state.npz');d=restore(m,z)
    cube=m.geom('cube').id;B=d.geom_xmat[cube].reshape(3,3).copy();C=d.geom_xpos[cube].copy();base=d.qpos.copy();pd=d.ctrl-base[:16]
    mf=m.geom('mf_tip').id;c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,mf}),key=lambda c:c.dist);G=d.geom_xmat[mf].reshape(3,3)
    tip_point=G.T@(c.pos-d.geom_xpos[mf]);tip_normal=G.T@(c.frame[:3]*(1 if int(c.geom1)==cube else -1))
    lo,hi,nlo,nhi=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1
    low=(np.ceil(lo[ids])+4-2048)*TICK-pd;high=(np.floor(hi[ids])-4-2048)*TICK-pd
    hand,support,check=inspect_for(m);records=[]
    for finger,active in [('if',np.arange(0,4)),('rf',np.arange(8,12))]:
      geom=m.geom(finger+'_tip').id
      for axis in (0,2):
       tangent=2 if axis==0 else 0
       for sign in (-1,1):
        best=None;began=time.perf_counter();starts=[base[active]]+[np.array([a,b,c,.8]) for a in (.4,1.) for b in (-.3,.3) for c in (.6,1.4)]
        for initial in starts:
            x0=np.r_[np.clip(initial,low[active]+1e-7,high[active]-1e-7),0.,0.]
            def metric(x):
                d.qpos[:]=base;d.qpos[active]=x[:4];d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d)
                local=np.zeros(3);local[axis]=sign*.0272;local[1]=x[4]/1000;local[tangent]=x[5]/1000
                G=d.geom_xmat[geom].reshape(3,3);normal=B[:,axis]*sign
                e=(d.geom_xpos[geom]+G@tip_point-(C+B@local))*1000;ne=G@tip_normal-normal;su,sp=check(d);pen=0.
                for contact in d.contact:
                    if cube in (int(contact.geom1),int(contact.geom2)):pen=max(pen,max(0.,-float(contact.dist)*1000))
                return e,ne,max(sp.values(),default=0.),max(su.values(),default=0.),pen
            def residual(x):
                e,ne,su,sp,pen=metric(x);return np.r_[e,ne*8,(x[:4]-base[active])*.03,x[4:]*.002,su*500,sp*500,max(0.,pen-.8)*100]
            fit=least_squares(residual,x0,bounds=(np.r_[low[active],-8.,-20.],np.r_[high[active],18.,20.]),max_nfev=130,ftol=1e-8,xtol=1e-8,gtol=1e-8)
            e,ne,su,sp,pen=metric(fit.x);err=float(np.linalg.norm(e));ang=float(np.degrees(np.arccos(np.clip(1-ne@ne/2,-1,1))))
            score=err+su*1000+sp*1000+max(0.,pen-1)*1000
            row=dict(finger=finger,face_axis=axis,face_sign=sign,tip_error_mm=err,normal_error_deg=ang,self_mm=su,support_mm=sp,cube_mm=pen,contact_height_mm=-float(fit.x[4]),along_face_mm=float(fit.x[5]),geometry_reachable=bool(err<.5 and ang<30 and su<=1e-6 and sp<=.15 and pen<=1),goal=motor(d.ctrl).tolist(),score=score)
            if best is None or score<best['score']:best=row
            if row['geometry_reachable']:break
        best['elapsed_seconds']=time.perf_counter()-began;records.append(best);print(json.dumps(best),flush=True)
    report=dict(status='postturn_third_contact_geometry_only',cases=records,reachable=[r for r in records if r['geometry_reachable']],hardware_execution_allowed=False,devices_opened=False,physical90_verified=False)
    (OUT/'postturn_third_contact_geometry.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
