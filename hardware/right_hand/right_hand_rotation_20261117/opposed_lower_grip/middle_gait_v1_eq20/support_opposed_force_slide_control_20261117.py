"""Separate real-contact preload from geometric fingertip sliding tasks."""
import numpy as np,mujoco
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS
from offline_goal_tracking_bounds_20261106 import tracking_increment_bounds


def increment(m,d,actual_allocation,geometry_tasks,planner,C,Ct,B,Bt,references,lower_loads,free_index_x=False,
              skip_index_force=True,ignored_force_names=(),equilibrium_control=None,equilibrium_weight=0.):
    active=np.r_[np.arange(8),np.arange(12,16)];ids=np.asarray(SIM_TO_MOTOR_IDS)-1
    translation=(Ct-C)*.12;length=np.linalg.norm(translation)
    if length>.0003:translation*=.0003/length
    omega=Rotation.from_matrix(Bt@B.T).as_rotvec()*.12;length=np.linalg.norm(omega)
    if length>.0075:omega*=.0075/length
    matrix=[];rhs=[];done=set();K=m.actuator_gainprm[:,0]
    actual=sorted(actual_allocation['contact_tasks'],key=lambda t:-t['actual_normal_N'])
    for task in actual:
        g=task['geom_id'];name=m.geom(g).name;point=np.asarray(task['point']);normal=np.asarray(task['outward_normal'])
        # Index distal motion is deliberately free to retreat from the cube.
        if name in ignored_force_names or (name=='if_tip' and skip_index_force):continue
        if name in references:
            local_n=normal@B;wanted_side=local_n[2]>.5 if name=='th_tip' else local_n[2]<-.5
            if not wanted_side or lower_loads[name]<=.02 or name in done:continue
            done.add(name)
        jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[g]))
        J=jp[:,active];row=normal@J
        expected=task['predicted_normal_N'];measured=task['actual_normal_N']
        if name in references:expected=references[name];measured=lower_loads[name]
        compliance=float(np.sum(row**2/K[active]))
        closing=float(np.clip((expected-measured)*compliance*.15,-.00008,.00008))
        desired=translation+np.cross(omega,point-C)
        matrix.append(row[None,:]*10000);rhs.append(np.array([normal@desired-closing])*10000)
    for task in geometry_tasks:
        g=task['geom_id'];name=m.geom(g).name;point=np.asarray(task['point']);normal=np.asarray(task['outward_normal'])
        jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[g]));J=jp[:,active]
        desired=translation+np.cross(omega,point-C)+.12*np.asarray(task['recover_tip_displacement'])
        if name=='if_tip' and free_index_x:
            axes=B[:,[1,2]].T
            matrix.append(axes@J*5000);rhs.append(axes@desired*5000)
        elif name in done:
            tangent=np.eye(3)-np.outer(normal,normal)
            matrix.append(tangent@J*2000);rhs.append(tangent@desired*2000)
        else:
            matrix.append(J*5000);rhs.append(desired*5000)
    if equilibrium_control is not None and equilibrium_weight>0:
        target_step=np.asarray(equilibrium_control)[active]-d.ctrl[active]
        matrix.append(np.eye(len(active))*equilibrium_weight);rhs.append(target_step*equilibrium_weight)
    matrix.append(np.eye(len(active))*.02);rhs.append(np.zeros(len(active)))
    A=np.vstack(matrix);v=np.concatenate(rhs);lower,upper=planner.increment_bounds()
    actual_ticks=np.empty(16);actual_ticks[ids]=d.qpos[:16]/TICK+2048
    lower,upper=tracking_increment_bounds(lower,upper,planner.continuous,actual_ticks,.12/TICK)
    lower=lower[ids[active]]*TICK;upper=upper[ids[active]]*TICK
    fixed=upper-lower<1e-12;change=np.zeros(len(active));change[fixed]=(lower[fixed]+upper[fixed])/2
    free=~fixed
    if np.any(free):
        change[free]=lsq_linear(A[:,free],v-A[:,fixed]@change[fixed],bounds=(lower[free],upper[free]),
                              tol=1e-10,max_iter=100).x
    delta=np.zeros(16);delta[ids[active]]=change/TICK
    return planner.advance(delta)
