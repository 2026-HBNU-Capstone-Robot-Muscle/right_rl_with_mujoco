"""Independent cube-loaded model replay of the completed EMPTY trajectory.

No devices. Synthetic cube registration is explicitly provisional. Separates
clearance/tracking from attaining the requested world-gravity yaw.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
from collections import Counter
import hashlib
import json
from pathlib import Path
import time
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from sac_cube_model import set_cube_physics
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS
from validate_first_cube_rotation_20261022 import projected_yaw

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261023'
SOURCE=ROOT/'right_hand_rotation_20261022/supported_yaw_result.json'
EMPTY=ROOT/'supported_yaw_empty_20261023_030552_675209/result.json'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--workers',type=int,default=4)
    parser.add_argument('--prefix',type=int,default=256);parser.add_argument('--quick',action='store_true')
    parser.add_argument('--scale',type=float,default=1.)
    args=parser.parse_args();OUT.mkdir(exist_ok=True)
    source=json.loads(SOURCE.read_bytes());empty=json.loads(EMPTY.read_bytes())
    assert empty['status']=='empty_supported_yaw_rehearsal_complete'
    assert empty['all_torque_off_verified'] and empty['settings_restored']
    base=np.array(source['base_goal_ticks']);goals=np.array(source['candidate_goal_ticks'])[:args.prefix]
    assert 0<args.scale<=1
    goals=np.rint(base+(goals-base)*args.scale).astype(int)
    assert 1<=len(goals)<=256 and len(goals)==args.prefix
    assert np.max(abs(np.diff(np.vstack([base,goals]),axis=0)))<=2
    assert np.max(abs(goals-base))<=256 and np.all(goals[:,[12,14,15]]<=base[[12,14,15]])
    physical=[s for s in empty['samples'] if s['phase']=='stage_4_supported_yaw_EMPTY']
    assert len(physical)==256 and np.array_equal(empty['final_on_state']['goal'],source['candidate_goal_ticks'][-1])
    actual=ticks_to_sim(physical[0]['actual'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    template,manifest=build_with_middle_thumb_hulls(env)
    gain0=template.actuator_gainprm.copy();bias0=template.actuator_biasprm.copy()
    hand={template.body('palm').id}
    for b in range(template.nbody):
        if int(template.body_parentid[b]) in hand:hand.add(b)
    support={template.geom(n).id for n in ('support_glass','support_wood')}
    cube=template.geom('cube').id;tips={template.geom(n).id for n in ('if_tip','mf_tip','rf_tip','th_tip')}
    up=-np.array(template.opt.gravity);up/=np.linalg.norm(up)
    qgoals=np.array([ticks_to_sim(q) for q in goals]);qbase=ticks_to_sim(base)
    joints=template.actuator_trnid[:,0]
    assert np.all(qgoals>=template.jnt_range[joints,0]-1e-12) and np.all(qgoals<=template.jnt_range[joints,1]+1e-12)
    assert np.all(qgoals>=template.actuator_ctrlrange[:,0]-1e-12) and np.all(qgoals<=template.actuator_ctrlrange[:,1]+1e-12)
    ranges=template.jnt_range.copy();controls=template.actuator_ctrlrange.copy()

    def simulate(job):
        reset,gain,friction,mass,measured,control=job
        m=copy.copy(template);set_cube_physics(m,side_m=.055,mass_kg=mass)
        m.actuator_gainprm[:]=gain0;m.actuator_biasprm[:]=bias0
        m.actuator_gainprm[:,0]*=gain;m.actuator_biasprm[:,1]*=gain
        m.geom_friction[cube]*=friction
        d=mujoco.MjData(m);d.qpos[:]=source['validation'][reset]['initial_qpos']
        if measured:d.qpos[:16]=actual
        d.ctrl[:]=qbase;mujoco.mj_forward(m,d)
        origin=d.qpos[-7:].copy();peaks=np.zeros(3);tracking=shift=drop=0.
        on_support=False;fault=None;hold=stable=0;contacts=[];trace=[];worst_motor=None
        path=np.repeat(qbase[None,:],len(goals),axis=0) if control else qgoals
        yaw=off=0.
        for k,goal in enumerate(list(path)+[path[-1]]*20):
            d.ctrl[:]=goal;active=False
            for _ in range(round(.05/m.opt.timestep)):
                mujoco.mj_step(m,d);contacts=[];active=False
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2);depth=max(0.,-float(c.dist)*1000)
                    if cube in (a,b):
                        other=b if a==cube else a;contacts.append(m.geom(other).name)
                        peaks[2]=max(peaks[2],depth);on_support|=other in support;active|=other in tips
                    elif a in support or b in support:peaks[1]=max(peaks[1],depth)
                    elif int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:peaks[0]=max(peaks[0],depth)
                errors=abs(d.qpos[:16]-goal);maximum=float(max(errors))
                if maximum>tracking:tracking=maximum;worst_motor=int(SIM_TO_MOTOR_IDS[int(np.argmax(errors))])
                shift=max(shift,float(np.linalg.norm(d.qpos[-7:-4]-origin[:3])*1000))
                drop=max(drop,float((origin[:3]-d.qpos[-7:-4])@up*1000))
            finite=bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
            if finite:
                yaw=projected_yaw(origin[3:],d.qpos[-4:],up)
                a=Rotation.from_quat(origin[3:][[1,2,3,0]]);b=Rotation.from_quat(d.qpos[-4:][[1,2,3,0]])
                rv=(b*a.inv()).as_rotvec()*180/np.pi;off=float(np.linalg.norm(rv-(rv@up)*up))
            if not finite:fault='nonfinite'
            elif on_support:fault='cube_on_external_support'
            elif peaks[0]>1e-6:fault='self_contact'
            elif peaks[1]>.15:fault='hand_support_contact'
            elif peaks[2]>1.5:fault='deep_cube_contact'
            elif tracking>.12:fault='tracking'
            elif shift>20 or drop>8:fault='cube_left_start_region'
            qualifies=not fault and abs(yaw+15)<=4 and off<=8 and shift<=12 and drop<=4 and active
            stable=stable+1 if k>=len(goals) and qualifies else 0;hold=max(hold,stable)
            if reset==0 and gain==friction==1 and mass==.1 and not control and k%2==0:
                trace.append(dict(time_s=(k+1)*.05,qpos=d.qpos.tolist()))
            if fault:break
        assert np.array_equal(ranges,m.jnt_range) and np.array_equal(controls,m.actuator_ctrlrange)
        return dict(reset=reset,gain=gain,cube_friction_scale=friction,mass_kg=mass,
            hand_initialization='measured_empty' if measured else 'original_synthetic',control=control,
            fault=fault,last_step=k+1,projected_yaw_deg=yaw,off_axis_deg=off,max_tracking_rad=tracking,
            maximum_tracking_motor_id=worst_motor,
            max_shift_mm=shift,drop_mm=drop,max_self_mm=peaks[0],max_hand_support_mm=peaks[1],max_cube_mm=peaks[2],
            cube_on_external_support=on_support,final_contacts=sorted(set(contacts)),qualified_hold_seconds=hold*.05,
            success=bool(not fault and hold*.05>=.5 and qualifies),trace=trace)

    variations=[(1.,1.,.1)] if args.quick else [(g,f,.1) for g in (.5,1.,1.5) for f in (.7,1.,1.3)]+[(1.,1.,m) for m in (.08,.12)]
    jobs=[(i,*v,measured,control) for measured in (False,True) for v in variations for i in range(7) for control in (False,True)]
    began=time.monotonic()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:rows=list(pool.map(simulate,jobs))
    controls_by={(r['reset'],r['gain'],r['cube_friction_scale'],r['mass_kg'],r['hand_initialization']):r for r in rows if r['control']}
    motion=[r for r in rows if not r['control']]
    for row in motion:
        key=(row['reset'],row['gain'],row['cube_friction_scale'],row['mass_kg'],row['hand_initialization'])
        baseline=controls_by[key]['projected_yaw_deg'];delta=row['projected_yaw_deg']-baseline
        row.update(no_motion_yaw_deg=baseline,yaw_beyond_control_deg=delta)
        row['success']&=delta<=-5
    report=dict(status='independent_loaded_supported_path_replay',
        source_candidate_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        completed_empty_source_sha256=hashlib.sha256(EMPTY.read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        goal_ticks=goals.tolist(),base_goal_ticks=base.tolist(),sample_period_s=.05,
        motion_samples=len(goals),motion_scale=args.scale,hold_samples=20,elapsed_seconds=time.monotonic()-began,
        clearance_tracking_screen_passed=all(r['fault'] is None for r in motion),
        requested_rotation_passed=all(r['success'] for r in motion),successful_cases=sum(r['success'] for r in motion),
        total_motion_cases=len(motion),total_control_cases=len(rows)-len(motion),
        fault_counts=dict(Counter(r['fault'] or 'none' for r in motion)),
        model_manifest=manifest,native_ranges_unchanged=True,real_cube_registration_verified=False,
        algorithm='offline_CEM_fixed_path_NOT_SAC',live_sac=False,hardware_commands_sent=0,
        hardware_execution_allowed=False,trials=rows)
    scale_suffix='' if args.scale==1 else f'_scale_{round(args.scale*100):03d}'
    target=OUT/f'supported_loaded_prefix_{len(goals)}{scale_suffix}{"_quick" if args.quick else ""}.json'
    target.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('goal_ticks','base_goal_ticks','model_manifest','trials')},indent=2),flush=True)
    return 0 if report['clearance_tracking_screen_passed'] else 2

if __name__=='__main__':raise SystemExit(main())
