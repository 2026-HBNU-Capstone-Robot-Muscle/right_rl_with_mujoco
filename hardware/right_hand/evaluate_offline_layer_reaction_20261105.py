"""Hypothetical layer-reaction torque on the free rigid cube, offline only.

The cube is rigid: this measures lower-body torque resistance, not an actual
articulated top-layer turn. Applied loads are assumed, not user measurements.
Every original dynamic safety bound is checked at the unchanged 1ms step.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore,inspect_for,native,angle
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from cube_contact_material_policy_20261029 import apply_material
from dynamic_common_grip_search_20261105 import CP,ROOT,R

def evaluate_reaction(m,z,torque_nm,save=None,cache=None):
    d=restore(m,z);cube=m.geom('cube').id;body=m.body('cube').id;hand,support,check=inspect_for(m);lo,hi,nlo,nhi=native(m);C0=d.qpos[-7:-4].copy();B0=d.xmat[body].reshape(3,3).copy();axis=-B0[:,1];control=d.ctrl.copy();worst=dict(self_mm=0.,support_mm=0.,cube_mm=0.,tracking_rad=0.,range_rad=0.);gap=longest=0.;stop=None;maxyaw=maxshift=maxtilt=0.;minclear=float('inf');trace=[];count=0;contacts={};lower=0;contactsteps=0;cache=precompute(m,hand) if cache is None else cache
    for sample in range(134):
      for sub in range(75):
        t=(sample*75+sub)*.001
        fraction=0. if t<1 else min(1.,t-1) if t<2 else 1. if t<8 else max(0.,1-(t-8)/2)
        d.xfrc_applied[body,3:]=axis*torque_nm*fraction
        mujoco.mj_step(m,d);count+=1;su,sp=check(d);worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.));worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))));worst['range_rad']=max(worst['range_rad'],float(max(0,max(nlo-d.qpos[:16]),max(d.qpos[:16]-nhi))))
        contacts={};inhand=external=False;lowercount=0;B=d.xmat[body].reshape(3,3);C=d.qpos[-7:-4]
        for ci,c in enumerate(d.contact):
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            other=b if a==cube else a
            if int(m.geom_bodyid[other]) in hand:
                inhand|=c.dist<.0004;worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(c.dist)*1000));ff=np.zeros(6);mujoco.mj_contactForce(m,d,ci,ff);name=m.geom(other).name;contacts[name]=contacts.get(name,0.)+float(ff[0]);height=-float((c.pos-C)@B[:,1])*1000;lowercount+=bool(height<55/6 and ff[0]>.05)
            elif other in support and c.dist<.0004:external=True
        gap=0 if inhand else gap+.001;longest=max(longest,gap);lower+=bool(lowercount>=2);contactsteps+=1;yaw=abs(angle(B,B0,axis));shift=float(np.linalg.norm(C-C0)*1000);tilt=float(np.degrees(np.arccos(np.clip(B[:,1]@B0[:,1],-1,1))));maxyaw=max(maxyaw,yaw);maxshift=max(maxshift,shift);maxtilt=max(maxtilt,tilt)
        if not np.array_equal(d.ctrl,control):raise RuntimeError('reaction probe changed targets')
        if worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25 or shift>80:
            stop='unchanged contact/range/tracking/retention bound';break
      cl=gaps(m,d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3),cache,True);minclear=min(minclear,min(v for name,v in cl))
      if sample%13==0 or sample==133 or stop:trace.append(dict(time_s=(sample+1)*.075,load_fraction=fraction,yaw_change_deg=angle(d.xmat[body].reshape(3,3),B0,axis),shift_mm=shift,upper_gap_mm=min(v for name,v in cl),contact_N=contacts.copy()))
      if stop:break
    if save:np.savez_compressed(save,qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    return dict(hypothetical_reaction_torque_nm=torque_nm,axis_world=axis.tolist(),safety_passed=stop is None,stop=stop,steps=count,max_yaw_change_deg=maxyaw,max_center_shift_mm=maxshift,max_tilt_change_deg=maxtilt,minimum_upper_gap_sampled_75ms_mm=minclear,upper_gap_sampling_is_not_continuous_proof=True,worst=worst,at_least_two_lower_contacts_fraction=lower/max(1,contactsteps),constant_targets=True,load_filter_passed=bool(stop is None and maxyaw<3 and maxshift<2 and maxtilt<3 and minclear>.3),trace=trace)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--torque',type=float,nargs='+',default=[.001,.003,.005,.01,.02]);ap.add_argument('--friction',type=float,default=.3);ap.add_argument('--source',type=Path);ap.add_argument('--tag',default='resting_release');a=ap.parse_args()
    if not all(0<=v<=.05 for v in a.torque) or a.friction not in (.2,.3,.5):raise ValueError('bounded hypothetical load matrix')
    source=a.source or ROOT/'middle_release/thumb_backoff32/fraction0.5.npz';out=ROOT/f'layer_reaction/{a.tag}_f{a.friction:g}';out.mkdir(parents=True,exist_ok=True);m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(source);hand,_,_=inspect_for(m);apply_material(m,a.friction,hand,m.geom('cube').id);cache=precompute(m,hand);cases=[]
    for magnitude in a.torque:
      for sign in (-1,1):
        began=time.perf_counter();row=evaluate_reaction(m,z,magnitude*sign,out/f'torque{magnitude*sign:+g}.npz',cache);row['elapsed_seconds']=time.perf_counter()-began;cases.append(row);report=dict(status='hypothetical_layer_reaction_on_rigid_cube_only',source=str(source.relative_to(R)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),friction=a.friction,cases=cases,actual_left_hand_load_measured=False,articulated_upper_layer_not_simulated=True,hardware_execution_allowed=False,devices_opened=False,current_PID_native_collision_and_retention_bounds_unchanged=True)
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in row.items() if k!='trace'}),flush=True)
if __name__=='__main__':main()
