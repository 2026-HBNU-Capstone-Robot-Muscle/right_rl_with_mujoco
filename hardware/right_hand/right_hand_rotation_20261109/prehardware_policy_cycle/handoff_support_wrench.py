"""Model-only cube contact forces, checked against generalized constraints."""
import numpy as np
import mujoco


def contact_wrench(m,d,hand,verify=False):
    cube=m.geom('cube').id;body=int(m.geom_bodyid[cube]);center=d.xipos[body]
    force=np.zeros(3);torque=np.zeros(3);by_geom={}
    for ci,c in enumerate(d.contact):
        a,b=int(c.geom1),int(c.geom2)
        if cube not in (a,b):continue
        other=b if a==cube else a
        if int(m.geom_bodyid[other]) not in hand:continue
        local=np.zeros(6);mujoco.mj_contactForce(m,d,ci,local)
        frame=c.frame.reshape(3,3);sign=-1. if a==cube else 1.
        F=sign*(frame.T@local[:3]);M=sign*(frame.T@local[3:])+np.cross(c.pos-center,F)
        force+=F;torque+=M;name=m.geom(other).name
        by_geom.setdefault(name,dict(normal_N=0.,force_world_N=np.zeros(3)))
        by_geom[name]['normal_N']+=float(local[0]);by_geom[name]['force_world_N']+=F
    Jp=np.zeros((3,m.nv));Jr=np.zeros((3,m.nv));mujoco.mj_jacBodyCom(m,d,Jp,Jr,body)
    joint=int(m.body_jntadr[body]);adr=int(m.jnt_dofadr[joint]);dofs=slice(adr,adr+6)
    predicted=Jp.T@force+Jr.T@torque
    error=float(np.max(abs(predicted[dofs]-d.qfrc_constraint[dofs])))
    if verify and error>1e-7:raise AssertionError(('Cube contact-force convention mismatch',error))
    mass=float(m.body_mass[body]);net=force+mass*m.opt.gravity
    up=-m.opt.gravity/max(np.linalg.norm(m.opt.gravity),1e-12)
    return dict(force_world_N=force,torque_world_Nm=torque,net_force_with_gravity_N=net,
                upward_support_N=float(force@up),weight_N=float(mass*np.linalg.norm(m.opt.gravity)),
                constraint_force_error=error,by_geom=by_geom)
