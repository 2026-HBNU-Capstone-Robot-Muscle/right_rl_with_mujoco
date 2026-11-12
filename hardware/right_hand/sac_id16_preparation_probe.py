"""Native EMPTY preparation audit from the last measured preflight, no hardware."""
import json
import argparse
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from sac_supported_start_env import SupportedStart
ROOT=Path(__file__).resolve().parent
TICK=2*np.pi/4096
ORDER=np.array(list(range(4,16))+list(range(4)))

def main():
    p=argparse.ArgumentParser();p.add_argument('--ordered',action='store_true');args=p.parse_args()
    evidence=ROOT/'leap_sac_path_empty_20261014_194239_306843.json'
    hw=json.loads(evidence.read_text(encoding='utf-8'))
    raw=np.array(hw['initial']['position']);target=np.array(json.loads((ROOT/'sac_reachable_empty_path.json').read_text())['start_ticks'])
    target[15]=3238
    relief=raw.copy();relief[[4,8,12]]=np.maximum(relief[[4,8,12]],int(np.ceil((np.pi+.65)/TICK)))
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=SupportedStart(registry.load('LeapCubeFaceUp',config=cfg),diagnostic_cube_side_mm=55);m=env.mj_model
    m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
    joint=int(m.actuator_trnid[11,0]);m.jnt_range[joint,1]=3254*TICK-np.pi
    support={m.geom(n).id for n in ('support_glass','support_wood')};rows=[]
    options=[('simultaneous',None)] if not args.ordered else [('thumb_first',range(4)),('ring_last',range(12)),('fingers_first',range(4,16))]
    for label,first_ids in options:
      for period in (.03,.04):
        d=mujoco.MjData(m);d.qpos[:]=np.array(json.loads((ROOT/'sac_cube55_resets.json').read_text())['candidates'][0]['qpos'])
        d.qpos[:16]=raw[ORDER]*TICK-np.pi;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
        phases=[relief]
        if first_ids is not None:
            midway=relief.copy();midway[list(first_ids)]=target[list(first_ids)];phases.append(midway)
        phases.extend([target,target])
        for stage,goalraw in enumerate(phases):
            start=d.qpos[:16].copy();goal=goalraw[ORDER]*TICK-np.pi;maxerr=0.;pen=0.;worst={}
            for step in range(1,71):
                ctrl=np.rint((start+(goal-start)*step/70+np.pi)/TICK)*TICK-np.pi;d.ctrl[:]=ctrl
                for _ in range(round(period/m.opt.timestep)):
                    mujoco.mj_step(m,d)
                    pen=max(pen,max([-float(c.dist)*1000 for c in d.contact if c.geom1 in support or c.geom2 in support]or[0]))
                err=np.abs(d.qpos[:16]-ctrl);i=int(np.argmax(err))
                if err[i]>maxerr:
                    maxerr=float(err[i]);worst=dict(id=int(ORDER[i]+1),step=step,goal=float(ctrl[i]),actual=float(d.qpos[i]),
                        contacts=[dict(a=m.geom(c.geom1).name,b=m.geom(c.geom2).name,penetration_mm=-float(c.dist)*1000) for c in d.contact if c.dist<0])
            for _ in range(round(.3/m.opt.timestep)):mujoco.mj_step(m,d)
            row=dict(order=label,period_seconds=period,stage=stage,max_tracking_rad=maxerr,support_penetration_mm=pen,
                     final_id16_ticks=float((d.qpos[11]+np.pi)/TICK),worst=worst,passed=bool(maxerr<=.18 and pen<=.5 and np.isfinite(d.qpos).all()))
            rows.append(row);print(json.dumps(row),flush=True)
        if args.ordered:
            # Continue from the actual prepared model state, without resetting joints.
            path=json.loads((ROOT/'sac_reachable_empty_path.json').read_text())
            maxerr=0.;pen=0.
            for goalraw in path['goals_id_order']:
                goalraw=np.array(goalraw);goalraw[15]=min(goalraw[15],3238);goal=goalraw[ORDER]*TICK-np.pi;d.ctrl[:]=goal
                for _ in range(round(.05/m.opt.timestep)):
                    mujoco.mj_step(m,d)
                    pen=max(pen,max([-float(c.dist)*1000 for c in d.contact if c.geom1 in support or c.geom2 in support]or[0]))
                maxerr=max(maxerr,float(np.max(np.abs(d.qpos[:16]-goal))))
            row=dict(order=label,period_seconds=period,stage='complete_empty_path_after_preparation',
                     max_tracking_rad=maxerr,support_penetration_mm=pen,passed=bool(maxerr<=.18 and pen<=.5 and np.isfinite(d.qpos).all()))
            rows.append(row);print(json.dumps(row),flush=True)
    output='sac_id16_preparation_ordered_result.json' if args.ordered else 'sac_id16_preparation_probe_result.json'
    (ROOT/output).write_text(json.dumps(dict(
        evidence=evidence.name,status='offline_hypothetical_preparation',motor_commands_sent=0,hardware_authorized=False,
        id16_goal_cap_ticks=3238,id16_assumed_upper_ticks=3254,results=rows,all_passed=all(r['passed'] for r in rows)),indent=2)+'\n')

if __name__=='__main__':main()
