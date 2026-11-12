"""Offline free-cube yaw evaluation of the collision-screened teacher segment.

The free cube is never rotated or translated by a goal. The hand trajectory is
prescribed, so this is a contact hypothesis test, not a fitted motor controller.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse
import json
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from adapt_original_sequence_clearance_20261024 import build, OUT
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--extended',action='store_true');args=parser.parse_args()
    input_name='horizontal_reference_primitive_to6.json' if args.extended else 'horizontal_reference_primitive.json'
    path=json.loads((OUT/input_name).read_bytes())
    assert path['static_path_clearance_satisfied']
    goals=np.array(path['goal_ticks']);qpath=ticks_to_sim(goals)
    m,manifest=build();set_cube_physics(m,side_m=.055,mass_kg=.1)
    cube=m.geom('cube').id;body=int(m.geom_bodyid[cube]);up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    forward=np.array([1.,0,0]);forward-=up*(up@forward);forward/=np.linalg.norm(forward)
    side=np.cross(up,forward);baseR=np.column_stack([forward,-up,np.cross(forward,-up)])
    q0=qpath[0];anchor=np.array([.115,.002,.020]);candidates=[]
    d=mujoco.MjData(m)
    def contact_bodies():
        names=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b) or c.dist>.0003:continue
            other=b if a==cube else a
            name=m.body(int(m.geom_bodyid[other])).name
            if name=='palm' or name.startswith(('if_','mf_','rf_','th_')):names.append(name)
        return sorted(set(names))
    def initialize(center,R,friction=.8):
        mujoco.mj_resetData(m,d);d.qpos[:16]=q0;d.ctrl[:]=q0
        d.qpos[-7:]=np.r_[center,Rotation.from_matrix(R).as_quat()[[3,0,1,2]]]
        m.geom_friction[cube,0]=friction;mujoco.mj_forward(m,d)
        overlap=max([-float(c.dist)*1000 for c in d.contact if cube in (int(c.geom1),int(c.geom2))]+[0])
        if overlap>.5:return None
        for _ in range(round(1.5/m.opt.timestep)):
            d.qpos[:16]=q0;d.qvel[:16]=0;d.ctrl[:]=q0;mujoco.mj_step(m,d)
        shift=float(np.linalg.norm(d.qpos[-7:-4]-center)*1000)
        contacts=contact_bodies()
        if not contacts or shift>30 or np.linalg.norm(d.qvel[-6:-3])>.02:return None
        return dict(initial_overlap_mm=overlap,settling_shift_mm=shift,settled_hand_contacts=contacts,
            settled_cube_center_m=d.qpos[-7:-4].tolist())
    for x in (-15,0,15):
        for y in (-20,0,20):
            for z in (0,12):
                for yaw in (0,45):
                    center=anchor+(forward*x+side*y+up*z)*.001
                    R=Rotation.from_rotvec(up*np.radians(yaw)).as_matrix()@baseR
                    ok=initialize(center,R)
                    if ok:candidates.append(dict(center_m=center.tolist(),R=R.tolist(),offset_mm=[x,y,z],yaw_deg=yaw,**ok))
    candidates.sort(key=lambda c:c['settling_shift_mm'])
    # Keep spatial diversity instead of many heights that settle to one point.
    selected=[]
    for c in candidates:
        if all(np.linalg.norm(np.array(c['settled_cube_center_m'])-s['settled_cube_center_m'])>.004 or c['yaw_deg']!=s['yaw_deg'] for s in selected):
            selected.append(c)
        if len(selected)>=8:break
    print('PLACEMENT_SCREEN','stable',len(candidates),'selected',len(selected),flush=True)
    def run(case,dt,friction=.8,record=False):
        ok=initialize(np.array(case['center_m']),np.array(case['R']),friction)
        if ok is None:return dict(valid_start=False,friction=friction,control_dt_s=dt)
        R0=d.xmat[body].reshape(3,3).copy();p0=d.qpos[-7:-4].copy();green0=R0@np.array([0.,-1,0])
        horizontal=R0[:,0]-up*(up@R0[:,0]);horizontal/=np.linalg.norm(horizontal)
        times=np.arange(len(qpath))*dt;duration=times[-1];elapsed=0.;next_observation=0.;samples=[];frames=[]
        targets={sign:Rotation.from_rotvec(up*sign*np.pi/2).as_matrix()@R0 for sign in (-1,1)}
        while elapsed<=duration+3.:
            q=np.array([np.interp(elapsed,times,qpath[:,j]) for j in range(16)])
            nxt=np.array([np.interp(elapsed+m.opt.timestep,times,qpath[:,j]) for j in range(16)])
            d.qpos[:16]=q;d.qvel[:16]=(nxt-q)/m.opt.timestep;d.ctrl[:]=q;mujoco.mj_step(m,d);elapsed+=m.opt.timestep
            if elapsed>=next_observation:
                R=d.xmat[body].reshape(3,3).copy();h=R[:,0]-up*(up@R[:,0])
                heading=None if np.linalg.norm(h)<.05 else float(np.degrees(np.arctan2(up@np.cross(horizontal,h),horizontal@h)))
                contacts=contact_bodies();shift=float(np.linalg.norm(d.qpos[-7:-4]-p0)*1000)
                samples.append(dict(elapsed_s=float(elapsed),path_fraction=min(1.,elapsed/duration),
                    yaw_heading_change_deg=heading,green_normal_tilt_change_deg=float(np.degrees(np.arccos(np.clip((R@np.array([0.,-1,0]))@green0,-1,1)))),
                    ninety_yaw_target_error_deg=min(float(np.degrees(Rotation.from_matrix(R@target.T).magnitude())) for target in targets.values()),
                    hand_contacts=contacts,displacement_mm=shift,retained_model=bool(contacts and shift<100),
                    cube_center_m=d.qpos[-7:-4].tolist()))
                if record:frames.append(dict(elapsed_s=float(elapsed),qpos=d.qpos.tolist()))
                next_observation+=.25
        final=samples[-1];retained=[s for s in samples if s['retained_model']]
        return dict(valid_start=True,placement=case,friction=friction,control_dt_s=dt,
            mode='prescribed_hand_free_cube',actual_motor_response_identified=False,
            final=final,samples=samples,best_retained_yaw_target_error_deg=min([s['ninety_yaw_target_error_deg'] for s in retained] or [180]),
            final_three_second_hold_retained=all(s['retained_model'] for s in samples if s['path_fraction']==1.),
            frames=frames if record else None)
    results=[]
    for i,case in enumerate(selected):
        for dt in (.05,.005):
            row=run(case,dt);results.append(row)
            print('YAW_REFERENCE',i,'dt',dt,'valid',row['valid_start'],
                'best_target_error',round(row.get('best_retained_yaw_target_error_deg',180),2),
                'final_hold_retained',row.get('final_three_second_hold_retained',False),flush=True)
    slow=[r for r in results if r['valid_start'] and r['control_dt_s']==.05]
    slow.sort(key=lambda r:(not r['final_three_second_hold_retained'],r['final']['ninety_yaw_target_error_deg']))
    sensitivity=[]
    if slow:
        for friction in (.4,.8,1.2):
            sensitivity.append(run(slow[0]['placement'],.05,friction,record=friction==.8))
    report=dict(source_primitive=input_name,cube_size_mm=55,cube_mass_g=100,
        stable_placement_cases=candidates,selected_placements=selected,runs=results,sensitivity=sensitivity,
        fast_reference_exceeds_current_hardware_goal_step_rate=True,support=manifest,
        robust_ninety_degree_yaw_candidate_found=bool(sensitivity and all(r.get('valid_start') and r['final_three_second_hold_retained']
            and r['final']['ninety_yaw_target_error_deg']<15 for r in sensitivity)),
        static_path_only_approved_for_offline_analysis=True,actual_cube_friction_unmeasured=True,
        hardware_execution_allowed=False,hardware_commands_sent=0,physical_ninety_degree_rotation_verified=False)
    output_name='horizontal_cube_dynamics_to6.json' if args.extended else 'horizontal_cube_dynamics.json'
    (OUT/output_name).write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('SUMMARY',json.dumps(dict(stable_placements=len(candidates),runs=len(results),
        robust_ninety_yaw_candidate=report['robust_ninety_degree_yaw_candidate_found'],
        best_slow_final=(slow[0]['final'] if slow else None))),flush=True)


if __name__=='__main__':main()
