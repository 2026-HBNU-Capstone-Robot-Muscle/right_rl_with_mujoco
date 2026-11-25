"""Offline infeasibility diagnosis. Relaxed LPs never produce motion targets."""
from pathlib import Path
import json,sys
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from scipy.optimize import linprog
from research_rotation_body_hold_20261104 import restore,inspect_for,native
from opposed_grip_force_balance_20261116 import allocate

parent=ROOT/'right_hand_rotation_20261116/opposed_lower_grip'
model=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':model.read_bytes()})
z=dict(np.load(parent/'independent_initial.npz',allow_pickle=False))
z.update(dict(np.load(parent/'index_base_support/equal14/static_proposal.npz',allow_pickle=False)))
d=restore(m,z);hand,_,_=inspect_for(m);cube=m.geom('cube').id;lo,hi,nlo,nhi=native(m)
saved={}
def trace(frame,event,arg):
    if frame.f_code is allocate.__code__:
        if event=='return':saved.update(frame.f_locals)
        return trace
sys.settrace(trace)
try:allocate(m,d,cube,hand,nlo,nhi,lo,hi)
finally:sys.settrace(None)
A,b,W,target=saved['A'],saved['bounds'],saved['W'],saved['target'];n=saved['count'];rows=[]
for label,lower in [('current',.6),('light_cage',.25),('no_requested_tip_pressure',0.)]:
    rhs=b.copy();rhs[-3:-1]=-lower
    fit=linprog(np.ones(n),A_ub=A,b_ub=rhs,A_eq=W,b_eq=target,bounds=[(0,3.5)]*n,method='highs')
    rows.append(dict(label=label,feasible=bool(fit.success),
        normal_N=None if not fit.success else {k:float(v) for k,v in zip(saved['groups'],saved['G']@fit.x)}))
fit=linprog(np.ones(n),A_ub=A[32:],b_ub=b[32:],A_eq=W,b_eq=target,bounds=[(0,3.5)]*n,method='highs')
rows.append(dict(label='diagnostic_remove_joint_torque_constraints',feasible=bool(fit.success),accepted=False))
K=saved['K'];bias=saved['bias'];J=saved['J']
torque_low=np.maximum(-.18*K,saved['flo'])-bias
torque_high=np.minimum(.18*K,saved['fhi'])-bias
act_A=np.vstack([J,-J,A[32:]]);act_b=np.r_[torque_high,-torque_low,b[32:]]
fit=linprog(np.ones(n),A_ub=act_A,b_ub=act_b,A_eq=W,b_eq=target,bounds=[(0,3.5)]*n,method='highs')
rows.append(dict(label='diagnostic_keep_torque_remove_goal_range',feasible=bool(fit.success),accepted=False))
active=np.r_[np.arange(4),np.arange(12,16)]
active_A=np.vstack([J[active],-J[active],A[32:]]);active_b=np.r_[saved['upper'][active],-saved['lower'][active],b[32:]]
fit=linprog(np.ones(n),A_ub=active_A,b_ub=active_b,A_eq=W,b_eq=target,bounds=[(0,3.5)]*n,method='highs')
rows.append(dict(label='diagnostic_constraints_on_index_thumb_only',feasible=bool(fit.success),accepted=False))
from hand_motor_mapping import SIM_TO_MOTOR_IDS
zero_bad=[dict(id=SIM_TO_MOTOR_IDS[i],low=float(saved['lower'][i]),high=float(saved['upper'][i]),bias=float(bias[i]))
          for i in range(16) if np.max(abs(J[i]))<1e-10 and not saved['lower'][i]<=0<=saved['upper'][i]]
report=dict(cube_basis=d.geom_xmat[cube].reshape(3,3).tolist(),world_up=z['up'].tolist(),gravity=m.opt.gravity.tolist(),
    palm_type=int(np.asarray(m.geom('palm_collision_1').type).item()),palm_size=m.geom('palm_collision_1').size.tolist(),
    feasibility=rows,unloaded_joint_conflicts=zero_bad,relaxed_diagnostics_do_not_authorize_goals=True,devices_opened=False)
print(json.dumps(report),flush=True)
(parent/'balance_diagnostic_20261117.json').write_text(json.dumps(report,indent=2)+'\n')
