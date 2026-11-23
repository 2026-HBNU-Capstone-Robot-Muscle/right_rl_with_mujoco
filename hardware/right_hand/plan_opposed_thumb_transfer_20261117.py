"""Static geometry/force feasibility for thumb shift before middle regrasp."""
from pathlib import Path
import json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from scipy.optimize import least_squares
from opposed_grip_force_balance_20261116 import allocate
from research_rotation_body_hold_20261104 import restore,inspect_for,native
from audit_opposed_working_grip_20261117 import MODEL,sha
from hand_motor_mapping import TICK
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate

BASE=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
source=BASE/'core_support_chain_10s/audit_end_state.npz'
out=BASE/'thumb_transfer_feasibility';out.mkdir(exist_ok=False)
m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
z=dict(np.load(source,allow_pickle=False));d=restore(m,z);hand,_,check=inspect_for(m)
lo,hi,nlo,nhi=native(m);cube=m.geom('cube').id;tip=m.geom('th_tip').id
C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();base=d.qpos.copy();pd=d.ctrl-d.qpos[:16]
c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}),key=lambda c:c.dist)
local=d.geom_xmat[tip].reshape(3,3).T@(c.pos-d.geom_xpos[tip]);clocal=B.T@(c.pos-C)
active=np.arange(12,16);low=np.maximum(nlo[active]+2*TICK,nlo[active]+2*TICK-pd[active]);high=np.minimum(nhi[active]-2*TICK,nhi[active]-2*TICK-pd[active])
geometry=precompute(m,hand);seed=np.clip(base[active],low+1e-9,high-1e-9);cases=[];began=time.perf_counter()
for x_mm in (9.,3.,-5.):
    targetlocal=clocal.copy();targetlocal[0]=x_mm/1000;targetlocal[2]=.02748
    target=C+B@targetlocal
    def measure(x,exact=False):
        d.qpos[:]=base;d.qpos[active]=x;d.qvel[:]=0;d.ctrl[:]=d.qpos[:16]+pd;mujoco.mj_forward(m,d)
        point=d.geom_xpos[tip]+d.geom_xmat[tip].reshape(3,3)@local
        su,sp=check(d);clear=gaps(m,d,C,B,geometry,exact)
        pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return point,clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen
    def residual(x):
        point,clear,selfmm,supportmm,pen=measure(x)
        return np.r_[(point-target)*3000,np.maximum(.5-np.asarray([v for _,v in clear]),0)*10,
                     (x-base[active])*.1,selfmm*500000,max(0.,supportmm-.05)*1000,max(0.,pen-.2)*1000]
    fit=least_squares(residual,seed,bounds=(low,high),max_nfev=150,ftol=1e-9,xtol=1e-9,gtol=1e-9)
    point,clear,selfmm,supportmm,pen=measure(fit.x,True);cert=SweepCertificate(m,hand,.3);gap,bad=cert.query(d,C,B)
    contact=any({int(c.geom1),int(c.geom2)}=={cube,tip} and c.dist<.0004 for c in d.contact)
    geometry_ok=bool(contact and not bad and selfmm<=1e-6 and supportmm<=.15 and pen<=1)
    control=force=None
    if geometry_ok:
        control,force=allocate(m,d,cube,hand,nlo,nhi,lo,hi,tip_minimum=.6,
            required_tips=('if_tip','th_tip'),normal_bounds={'mf_tip':(0.,.1)})
    case=dict(thumb_target_x_mm=x_mm,point_error_mm=float(np.linalg.norm(point-target)*1000),
        geometry_passed=geometry_ok,upper_gap_mm=gap,self_mm=selfmm,support_mm=supportmm,cube_mm=pen,
        middle_unload_feasible=control is not None,force=force)
    if control is not None:np.savez_compressed(out/f'thumb_x{x_mm:g}_proposal.npz',qpos=d.qpos,ctrl=control)
    cases.append(case);print(json.dumps(case),flush=True)
report=dict(status='thumb_relocation_before_middle_unload_static',source_sha256=sha(source),model_sha256=sha(MODEL),
    cases=cases,static_planning_only=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
