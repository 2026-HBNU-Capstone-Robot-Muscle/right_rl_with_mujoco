"""Offline thumb pressure relief scout. No device access or motor entrypoint."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from sac_cube_model import set_cube_physics
from screen_loaded_repeat_20261026 import cached_contacts
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,ticks_to_sim
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/thumb_relief_scout'
SOURCE=ROOT/'right_hand_rotation_20261028/observed90_prefix638/full_loaded_27/loaded_path.npz'
def changed_path(motor_id,offset):
    z=np.load(SOURCE);goals=z['goals'].copy();phases=z['phases'];weights=np.ones(len(goals))
    grasp=np.flatnonzero(phases=='grasp');release=np.flatnonzero(phases=='release');tail=np.flatnonzero(phases=='remove_hold')
    weights[grasp]=np.linspace(0,1,len(grasp));weights[release]=np.linspace(1,0,len(release));weights[tail]=0
    goals[:,motor_id-1]+=np.rint(offset*weights).astype(int)
    rows=[];names=[];before=z['start']
    for phase,q in zip(phases,goals):
        count=max(1,int(np.ceil(np.max(abs(q-before))/2)))
        for n in range(1,count+1):rows.append(np.rint(before+(q-before)*n/count).astype(int));names.append(str(phase))
        before=q
    return z['start'].copy(),np.asarray(rows),np.asarray(names)
def main():
    began=time.perf_counter();OUT.mkdir(parents=True,exist_ok=True)
    prior=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes())
    up=np.asarray(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world'])
    seed=next(x for x in prior['loaded_physics'] if (x['book_front_mm'],x['gain_scale'],x['friction'])==(-10.,1.,.3))
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    start,_,_=changed_path(1,0);m,manifest=build(env,OUT/'scene.xml',book_projection_mm=-10.)
    correct_model(m,np.asarray(prior['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=.002;m.opt.gravity[:]=-9.81*up
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    cube=m.geom('cube').id;m.geom_friction[cube,0]=.3;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);lo,hi=limits();ids=np.asarray(SIM_TO_MOTOR_IDS)-1
    low,high=(lo[ids]-2048)*TICK,(hi[ids]-2048)*TICK;root=m.actuator('mf_mcp_act').id
    candidates=[(1,0),(1,-16),(1,16),(2,-16),(2,16)];reports=[]
    for motor,offset in candidates:
        start,goals,phases=changed_path(motor,offset);name=f'ID{motor}_{offset:+d}'
        if np.any(goals<lo) or np.any(goals>hi) or np.max(abs(np.diff(np.vstack([start,goals]),axis=0)))>2:raise RuntimeError('Candidate target bounds changed')
        np.savez_compressed(OUT/(name+'.npz'),start=start,goals=goals,phases=phases)
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(start);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=seed['settled_cube_qpos'];d.qvel[:]=0
        for _ in range(1000):mujoco.mj_step(m,d)
        center=d.qpos[-7:-4].copy();yawB=None;angle=tilt=None;stop=None;steps=0
        maxself=maxsupport=maxcube=maxtrack=maxrange=longest=nohand=0.;root_torque=[];root_tracking=[];normal_forces=[]
        for index,(q,phase) in enumerate(zip(goals,phases),1):
            d.ctrl[:]=ticks_to_sim(q);target=round(index*.075/.002)
            for _ in range(target-steps):
                mujoco.mj_step(m,d);su,sp=inspect(d);maxself=max(maxself,max(sp.values(),default=0.));maxsupport=max(maxsupport,max(su.values(),default=0.))
                maxtrack=max(maxtrack,float(np.max(abs(d.qpos[:16]-d.ctrl))));maxrange=max(maxrange,float(max(0.,max(low-d.qpos[:16]),max(d.qpos[:16]-high))))
                inhand=False;external=False;normal=0.
                for ci,c in enumerate(d.contact):
                    a,b=int(c.geom1),int(c.geom2)
                    if cube not in (a,b):continue
                    other=b if a==cube else a
                    if int(m.geom_bodyid[other]) in hand:
                        maxcube=max(maxcube,max(0.,-float(c.dist)*1000));inhand|=c.dist<.0004
                        force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);normal+=max(0.,float(force[0]))
                    elif other in support and c.dist<.0004:external=True
                nohand=0. if inhand else nohand+.002;longest=max(longest,nohand)
                if maxself>1e-6 or maxsupport>.15 or maxcube>1 or maxtrack>.18 or maxrange>1e-6 or external or longest>.25:stop='same_contact_range_tracking_retention_bound';break
            steps=target
            if phase in ('grasp_hold','yaw','yaw_hold'):
                root_torque.append(abs(float(d.actuator_force[root])));root_tracking.append(abs(float(d.qpos[m.joint('mf_mcp').qposadr[0]]-d.ctrl[root])));normal_forces.append(normal)
            if phase=='grasp_hold':yawB=d.xmat[m.body('cube').id].reshape(3,3).copy()
            if phase=='yaw_hold' and yawB is not None:
                R=d.xmat[m.body('cube').id].reshape(3,3);delta=Rotation.from_matrix(R@yawB.T).as_quat()
                angle=float(np.degrees(2*np.arctan2(delta[:3]@up,delta[3])));tilt=float(np.degrees(np.arccos(np.clip((R@np.array([0.,-1,0]))@(yawB@np.array([0.,-1,0])),-1,1))))
            if stop:break
        report=dict(candidate=name,motor_id=motor,offset_ticks=offset,offset_degrees=offset*360/4096,goals=len(goals),last_sample=index,last_phase=str(phase),
            passed=bool(stop is None and inhand and np.linalg.norm(d.qpos[-7:-4]-center)<.08),stop=stop,
            yaw_before_open_deg=angle,yaw_tilt_change_deg=tilt,max_self_mm=maxself,max_support_mm=maxsupport,max_cube_mm=maxcube,
            max_tracking_rad=maxtrack,max_native_range_excess_rad=maxrange,mean_model_middle_root_torque=float(np.mean(root_torque)) if root_torque else None,
            peak_model_middle_root_torque=max(root_torque,default=None),mean_model_total_cube_normal_force=float(np.mean(normal_forces)) if normal_forces else None,
            physical_force_calibrated=False,motor_writes=0)
        reports.append(report);print(json.dumps(report),flush=True)
        (OUT/'review.json').write_text(json.dumps(dict(status='offline_thumb_relief_scout_only',source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
            nominal_only=True,actuator_and_friction_parameters_unchanged=True,all_contact_forces_retained=True,
            real_force_calibration_verified=False,physical_execution_authorized=False,cases=reports,elapsed_seconds=time.perf_counter()-began),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
