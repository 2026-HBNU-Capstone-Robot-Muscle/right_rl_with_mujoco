"""Offline exact half-speed diagnostic, latest measured start + five resets."""
import hashlib
import argparse
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from sac_supported_start_env import SupportedStart
from empty_path_retime import half_speed

ROOT=Path(__file__).resolve().parent
TICK=2*np.pi/4096
ORDER=np.array(list(range(4,16))+list(range(4)))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--id12-hold',action='store_true')
    parser.add_argument('--id12-stiffness',action='store_true')
    parser.add_argument('--id12-backoff',action='store_true');args=parser.parse_args()
    if args.id12_backoff:args.id12_stiffness=True
    if args.id12_stiffness:args.id12_hold=True
    sources=['sac_reachable_empty_path_id16capped.json','leap_drive_readonly_result.json',
             'empty_path_retime.py','sac_cube55_resets.json']
    doc=json.loads((ROOT/sources[0]).read_text());hw=json.loads((ROOT/sources[1]).read_text())
    if hw['status']!='read_complete_not_motion_approval' or len(hw['motors'])!=16:
        raise ValueError('Incomplete measured start')
    motors=sorted(hw['motors'],key=lambda r:r['id'])
    if [r['id'] for r in motors]!=list(range(1,17)) or any(r['torque_enabled_end'] for r in motors):
        raise ValueError('Invalid measured start IDs/torque')
    raw=np.array([r['position_ticks_signed'] for r in motors]);target=np.array(doc['start_ticks'])
    goals=half_speed(target,doc['goals_id_order']);rows=[]
    if args.id12_hold:
        from id12_hold_spec import select_prefix,HOLD_SECONDS
        prefix=select_prefix(goals)
        goals=np.vstack([prefix,np.repeat(prefix[-1:],round(HOLD_SECONDS/.05),axis=0)])
        sources.append('id12_hold_spec.py')
        if args.id12_backoff:
            from id12_backoff_spec import targets
            back=targets(prefix[-1])
            goals=np.vstack([goals,back,np.repeat(back[-1:],round(HOLD_SECONDS/.05),axis=0)])
            sources.append('id12_backoff_spec.py')
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=SupportedStart(registry.load('LeapCubeFaceUp',config=cfg),diagnostic_cube_side_mm=55)
    m=env.mj_model;cube=m.geom('cube').id;m.geom_contype[cube]=m.geom_conaffinity[cube]=0
    if args.id12_stiffness:
        if m.joint(int(m.actuator_trnid[7,0])).name!='mf_dip':raise ValueError('ID12 mapping mismatch')
        # Sensitivity only: real P-to-torque conversion remains uncalibrated.
        m.actuator_gainprm[7,0]*=600/450
        m.actuator_biasprm[7,1]*=600/450
        sources.append('id12_stiffness_profile.py')
    m.jnt_range[int(m.actuator_trnid[11,0]),1]=3254*TICK-np.pi
    support={m.geom(n).id for n in ('support_glass','support_wood')}
    resets=json.loads((ROOT/'sac_cube55_resets.json').read_text())['candidates']
    def advance(d,ctrl,seconds):
        d.ctrl[:]=ctrl;pen=0.;err20=None
        for j in range(round(seconds/m.opt.timestep)):
            mujoco.mj_step(m,d)
            pen=max(pen,max([-float(c.dist)*1000 for c in d.contact if c.geom1 in support or c.geom2 in support] or [0]))
            if j+1==round(.02/m.opt.timestep):err20=float(np.max(np.abs(d.qpos[:16]-ctrl)))
        return max(float(np.max(np.abs(d.qpos[:16]-ctrl))),err20 or 0),pen
    for case in range(6):
        d=mujoco.MjData(m);d.qpos[:]=resets[max(0,case-1)]['qpos']
        stages=[]
        if case==0:
            d.qpos[:16]=raw[ORDER]*TICK-np.pi
            relief=raw.copy();relief[[4,8,12]]=np.maximum(relief[[4,8,12]],int(np.ceil((np.pi+.65)/TICK)))
            thumb=relief.copy();thumb[:4]=target[:4]
            stages=[relief,thumb,target,target]
        d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
        for stage,g in enumerate(stages):
            start=d.qpos[:16].copy();end=g[ORDER]*TICK-np.pi;mx=pen=0.
            for step in range(1,71):
                ctrl=np.rint((start+(end-start)*step/70+np.pi)/TICK)*TICK-np.pi
                e,p=advance(d,ctrl,.03);mx=max(mx,e);pen=max(pen,p)
            e,p=advance(d,ctrl,.3);mx=max(mx,e);pen=max(pen,p)
            rows.append(dict(case='latest_measured_start',stage=stage,max_tracking_rad=mx,
                max_support_penetration_mm=pen,passed=bool(mx<=.18 and pen<=.5 and np.isfinite(d.qpos).all())))
        mx=pen=0.
        for g in goals:
            e,p=advance(d,g[ORDER]*TICK-np.pi,.05);mx=max(mx,e);pen=max(pen,p)
        rows.append(dict(case='latest_measured_start' if case==0 else f'reset_{case-1}',stage='half_speed_empty_path',
            max_tracking_rad=mx,max_support_penetration_mm=pen,passed=bool(mx<=.18 and pen<=.5 and np.isfinite(d.qpos).all())))
        print(json.dumps(rows[-1]),flush=True)
    result=dict(status='offline_empty_only_diagnostic',results=rows,all_passed=all(r['passed'] for r in rows),
        time_scale=2,control_dt=.05,command_count=len(goals),duration_seconds=len(goals)*.05,
        path_max_step_ticks=int(np.max(np.abs(np.diff(np.vstack([target,goals]),axis=0)))),
        sources_sha256={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources},
        motor_commands_sent=0,loaded_flip_validated=False,live_policy=False,
        caveat='Native model only; actuator mismatch/contact cause remain unknown. ID16 upper3254 is hypothetical.')
    if args.id12_hold:
        result.update(status='offline_id12_fixed_hold_empty',prefix_count=len(prefix),hold_seconds=HOLD_SECONDS,
                      id12_hold_goal_ticks=3290,hold_rewrites_on_hardware=False)
    name='sac_id12_hold_audit_result.json' if args.id12_hold else 'sac_half_speed_empty_audit_result.json'
    if args.id12_stiffness:
        name='sac_id12_stiffness_sensitivity_result.json'
        result.update(status='offline_id12_gain_sensitivity_only',assumed_gain_ratio=600/450,
            calibrated_hardware_dynamics=False,hardware_current_cap_not_modeled=True)
    if args.id12_backoff:
        name='sac_id12_backoff_audit_result.json'
        result.update(status='offline_id12_backoff_empty',return_count=len(back),id12_return_goal_ticks=int(back[-1,11]))
    (ROOT/name).write_text(json.dumps(result,indent=2)+'\n')
    print('all_passed',result['all_passed'],flush=True)

if __name__=='__main__':main()
