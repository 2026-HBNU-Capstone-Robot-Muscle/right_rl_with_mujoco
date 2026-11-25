"""Actual model contact direction for an intended positive-yaw fingertip push."""
import numpy as np
import mujoco


def tip_yaw_contact(model,data,cube,tip,up):
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
    return dict(gap_mm=gap*1000,lever_mm=lever,normal_N=force,contacts=len(contacts),positive_contact_ready=bool(contacts and gap<.0002 and lever>3.))
