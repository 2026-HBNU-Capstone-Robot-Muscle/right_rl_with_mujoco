"""Assumed small torsion probes of rigid-cube holds; not a measured left hand."""
import json,time
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import CP,OUT,restore,angle,native,inspect_for
from cube_contact_material_policy_20261029 import apply_material

def main():
    sources=[('original',CP/'closest_state.npz'),('thumb8',OUT/'turn_0_lower_8_normal_10/end_state.npz'),('pair8_12',OUT/'lower_pair_m8_t12/end_state.npz'),('pair20_16',OUT/'lower_pair_m20_t16/end_state.npz')]
    reports=[]
    for name,path in sources:
      for gain in (.5,1.,1.5):
       for friction in (.2,.3,.5):
        began=time.perf_counter();m=mujoco.MjModel.from_binary_path(str(CP/'closest_model.mjb'));z=np.load(path);d=restore(m,z)
        m.actuator_gainprm[:,0]*=gain;m.actuator_biasprm[:,1]*=gain
        cube=m.geom('cube').id;body=m.body('cube').id;hand,support,check=inspect_for(m);apply_material(m,friction,hand,cube)
        lo,hi,nlo,nhi=native(m);up=z['up'];B=None;C=None;worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);stop=None;drift=shift=0.;longest=gap=0.;opposing=0;n=0
        for step in range(10000):
            t=step*.001;torque=.002 if 3<=t<5 else (-.002 if 7<=t<9 else 0.)
            d.xfrc_applied[body,3:]=up*torque;mujoco.mj_step(m,d)
            su,sp=check(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))))
            inhand=False;external=False;contact={}
            for ci,c in enumerate(d.contact):
                a,b=int(c.geom1),int(c.geom2)
                if cube not in (a,b):continue
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    inhand|=c.dist<.0004;worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000));f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);name_g=m.geom(other).name;contact[name_g]=contact.get(name_g,0.)+float(f[0])
                elif other in support and c.dist<.0004:external=True
            gap=0. if inhand else gap+.001;longest=max(longest,gap)
            if step>=2000:
                if B is None:B=d.xmat[body].reshape(3,3).copy();C=d.qpos[-7:-4].copy()
                n+=1;drift=max(drift,abs(angle(d.xmat[body].reshape(3,3),B,up)));shift=max(shift,float(np.linalg.norm(d.qpos[-7:-4]-C)*1000));opposing+=all(contact.get(g,0.)>.2 for g in ('mf_tip','th_tip'))
            if worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25:
                stop='unchanged contact/range/tracking/retention bound';break
        row=dict(source=name,gain_scale=gain,friction=friction,steps=step+1,safety_passed=stop is None,stop=stop,max_yaw_drift_deg=drift,max_center_shift_mm=shift,opposing_contact_fraction=opposing/max(1,n),hold_probe_passed=bool(stop is None and drift<3 and shift<2 and opposing/max(1,n)>.8),worst=worst,elapsed_seconds=time.perf_counter()-began)
        reports.append(row);report=dict(status='assumed_torsion_rigid_cube_hold_research',applied_probe_torque_Nm=.002,actual_left_hand_torque_measured=False,full_prefix_and_initial_seating_not_replayed=True,physical_hold_strength_verified=False,hardware_execution_allowed=False,devices_opened=False,cases=reports)
        (OUT/'torsion_stress_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(row),flush=True)
if __name__=='__main__':main()
