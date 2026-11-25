"""Offline path candidates inside the tested ID12 target envelope. No launcher."""
import json
import hashlib
import argparse
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from sac_supported_start_env import SupportedStart
from sac_forward_geometry import measure,eligible
ROOT=Path(__file__).resolve().parent
TICK=2*np.pi/4096
ORDER=np.array(list(range(4,16))+list(range(4)))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--refine',action='store_true');args=ap.parse_args()
    source=ROOT/'sac_reachable_empty_path_id16capped.json'
    doc=json.loads(source.read_text());raw=np.array([doc['start_ticks']]+doc['goals_id_order'])
    resets=json.loads((ROOT/'sac_cube55_resets.json').read_text())['candidates']
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=SupportedStart(registry.load('LeapCubeFaceUp',config=cfg),diagnostic_cube_side_mm=55)
    m=env.mj_model;support={m.geom(n).id for n in ('support_glass','support_wood')}
    cube=m.geom('cube').id;cube_masks=(m.geom_contype[cube],m.geom_conaffinity[cube])
    j12=int(m.actuator_trnid[7,0]);j16=int(m.actuator_trnid[11,0])
    assert m.joint(j12).name=='mf_dip' and m.joint(j16).name=='rf_dip'
    m.jnt_range[j16,1]=3254*TICK-np.pi
    def evaluate(targets,reset_index=0,loaded=True,upper12=3227):
        m.jnt_range[j12,1]=upper12*TICK-np.pi
        m.geom_contype[cube]=cube_masks[0] if loaded else 0;m.geom_conaffinity[cube]=cube_masks[1] if loaded else 0
        goals=targets[:,ORDER]*TICK-np.pi
        d=mujoco.MjData(m);d.qpos[:]=resets[reset_index]['qpos'];d.ctrl[:]=goals[0]
        d.qpos[7]=min(d.qpos[7],m.jnt_range[j12,1]);d.qpos[11]=min(d.qpos[11],m.jnt_range[j16,1])
        mujoco.mj_forward(m,d);initial=d.qpos[-7:].copy()
        longest=hold=0;maxerr=pen=selfpen=0.;maxshift=0.;worst=None
        for step,goal in enumerate(goals):
            d.ctrl[:]=goal
            for _ in range(round(.05/m.opt.timestep)):
                mujoco.mj_step(m,d)
                for c in d.contact:
                    if c.dist>=0:continue
                    depth=-float(c.dist)*1000
                    if c.geom1 in support or c.geom2 in support:pen=max(pen,depth)
                    elif cube not in (c.geom1,c.geom2):selfpen=max(selfpen,depth)
            errors=np.abs(d.qpos[:16]-goal);idx=int(np.argmax(errors));error=float(errors[idx])
            if error>maxerr:
                maxerr=error;worst=dict(step=step,motor_id=int(ORDER[idx]+1),goal_ticks=float((goal[idx]+np.pi)/TICK),actual_ticks=float((d.qpos[idx]+np.pi)/TICK))
            tip,axis,err=map(float,measure(initial[-4:],d.qpos[-4:],xp=np))
            shift=float(np.linalg.norm(d.qpos[-7:-4]-initial[:3]));maxshift=max(maxshift,shift)
            valid=eligible(tip,axis,err,shift,xp=np) and pen<=.5
            hold=hold+1 if valid else 0;longest=max(longest,hold)
        return dict(loaded=loaded,reset_index=reset_index,hypothetical_id12_actual_upper_ticks=upper12,
            longest_hold_steps=longest if loaded else None,max_tracking_rad=maxerr,
            worst_tracking=worst,
            max_support_penetration_mm=pen,max_self_penetration_mm=selfpen,
            final_target_error_deg=float(np.degrees(err)) if loaded else None,
            max_cube_center_shift_mm=maxshift*1000 if loaded else None,
            passed=bool(np.isfinite(d.qpos).all() and maxerr<=.18 and pen<=.5 and (not loaded or longest>=23)))
    rows=[];candidates=[]
    for cap in ((3260,3230,3200) if args.refine else (3290,3260,3230)):
      for pip_offset in ((.05,.075,.10,.125,.15) if args.refine else (-.20,-.10,0.,.10,.20)):
        targets=raw.copy();weight=np.clip((raw[:,11]-3000)/290.,0,1)
        targets[:,11]=np.minimum(targets[:,11],cap)
        targets[:,10]+=np.rint(weight*pip_offset/TICK).astype(int)
        if np.max(np.abs(np.diff(targets,axis=0)))>70:continue
        # Refine at the actual half-speed diagnostic cadence, preserving every
        # endpoint. Added targets are integer midpoints, not longer pauses.
        if args.refine:
            expanded=[targets[0]]
            for a,b in zip(targets[:-1],targets[1:]):expanded.extend([np.rint((a+b)/2).astype(int),b])
            targets=np.array(expanded)
        metrics=evaluate(targets,1,True,3210) if args.refine else evaluate(targets)
        case=dict(id12_goal_cap_ticks=cap,id11_offset_rad=pip_offset,half_speed=args.refine,**metrics)
        rows.append(case)
        print(json.dumps(case),flush=True)
        if metrics['passed']:candidates.append((targets,case))
    validation=[];selected=None
    for targets,case in sorted(candidates,key=lambda c:(abs(c[1]['id11_offset_rad']),c[1]['max_tracking_rad'])):
        checks=[evaluate(targets,i,loaded,upper) for upper in (3210,3227,3254) for loaded in (False,True) for i in range(5)]
        validation.append(dict(candidate=case,checks=checks,all_passed=all(r['passed'] for r in checks)))
        if validation[-1]['all_passed']:
            selected=dict(candidate=case,start_ticks=targets[0].tolist(),goals_id_order=targets[1:].tolist(),checks=checks);break
    result=dict(status='offline_id12_envelope_reference_search',source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        results=rows,validation=validation,selected=selected,motor_commands_sent=0,hardware_authorized=False,
        caveat='Hypothetical conservative reach limits from measured samples, not proven mechanical limits. Current/PID mapping and hand collision geometry remain uncalibrated. No new hardware path installed.')
    name='sac_id12_replan_refined_result.json' if args.refine else 'sac_id12_replan_probe_result.json'
    (ROOT/name).write_text(json.dumps(result,indent=2)+'\n')
    print('search_passes',len(candidates),'selected',selected is not None,flush=True)

if __name__=='__main__':main()
