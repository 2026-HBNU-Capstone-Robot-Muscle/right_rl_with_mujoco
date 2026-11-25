"""Actual model contact direction for an intended positive-yaw fingertip push."""
import numpy as np
import mujoco


def finger_yaw_moments(model,data,cube,finger,up,threshold=.05):
    """Actual contact contributions; opposing support is not automatically failure.

    Native collision/tracking/joint guards remain separate. Positive normal
    moment is only a local pushing-direction test, not a force-closure proof.
    """
    local=np.zeros(6);rows=[];normal_sum=total_sum=tip_sum=0.
    for i,c in enumerate(data.contact):
        if cube not in (int(c.geom1),int(c.geom2)):continue
        other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
        if not (model.body(int(model.geom_bodyid[other])).name or '').startswith(finger+'_'):continue
        mujoco.mj_contactForce(model,data,i,local)
        if local[0]<threshold:continue
        direction=-1. if int(c.geom1)==cube else 1.;frame=c.frame.reshape(3,3)
        normal_force=direction*frame[0]*local[0];force=direction*(frame.T@local[:3]);arm=c.pos-data.geom_xpos[cube]
        normal=float(np.cross(arm,normal_force)@up);total=float((np.cross(arm,force)+direction*(frame.T@local[3:]))@up)
        name=model.geom(other).name or str(other);normal_sum+=normal;total_sum+=total
        if name==finger+'_tip':tip_sum+=normal
        rows.append(dict(geom=name,normal_N=float(local[0]),normal_yaw_moment_Nm=normal,total_yaw_moment_Nm=total))
    return dict(normal_yaw_moment_Nm=normal_sum,total_yaw_moment_Nm=total_sum,tip_normal_yaw_moment_Nm=tip_sum,opposed_loaded_geoms=sorted({r['geom'] for r in rows if r['normal_yaw_moment_Nm']<=0}),contacts=rows,strict_any_opposed_rejected=any(r['normal_yaw_moment_Nm']<=0 for r in rows),net_direction_rejected=bool(rows and (normal_sum<=0 or any(r['geom']==finger+'_tip' for r in rows) and tip_sum<=0)))


def forwarded_state(model,data):
    query=mujoco.MjData(model)
    for key in ('qpos','qvel','ctrl','act','qacc_warmstart','mocap_pos','mocap_quat','xfrc_applied','qfrc_applied'):
        getattr(query,key)[:]=getattr(data,key)
    query.time=data.time;mujoco.mj_forward(model,query)
    return query


def tip_yaw_contact(model,data,cube,tip,up,forward_copy=True):
    if forward_copy:data=forwarded_state(model,data)
    points=np.zeros(6);gap=float(mujoco.mj_geomDistance(model,data,cube,tip,.2,points));contacts=[];wrench=np.zeros(6)
    for index,c in enumerate(data.contact):
        if {int(c.geom1),int(c.geom2)}!={cube,tip}:continue
        n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);mujoco.mj_contactForce(model,data,index,wrench)
        contacts.append(dict(lever_mm=float(np.cross(c.pos-data.geom_xpos[cube],-n)@up*1000),normal_N=max(0.,float(wrench[0]))))
    if contacts:
        force=sum(c['normal_N'] for c in contacts)
        lever=sum(c['normal_N']*c['lever_mm'] for c in contacts)/force if force>1e-7 else min(c['lever_mm'] for c in contacts)
    else:
        n=points[3:]-points[:3];length=np.linalg.norm(n)
        lever=float(np.cross(points[:3]-data.geom_xpos[cube],-n/max(length,1e-12))@up*1000);force=0.
    return dict(gap_mm=gap*1000,lever_mm=lever,normal_N=force,contacts=len(contacts),positive_contact_ready=bool(contacts and gap<.0002 and lever>3.),geometry_evaluated_on_forwarded_copy=forward_copy)
