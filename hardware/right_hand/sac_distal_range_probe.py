"""Offline whole-path target-range comparison; no per-motor live trial loop."""
import json
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
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=SupportedStart(registry.load('LeapCubeFaceUp',config=cfg),diagnostic_cube_side_mm=55);m=env.mj_model
    support={m.geom(n).id for n in ('support_glass','support_wood')}
    m.jnt_range[int(m.actuator_trnid[11,0]),1]=3254*TICK-np.pi
    doc=json.loads((ROOT/'sac_reachable_empty_path_id16capped.json').read_text())
    raw=np.array([doc['start_ticks']]+doc['goals_id_order'])
    resets=json.loads((ROOT/'sac_cube55_resets.json').read_text())['candidates'];rows=[]
    for cap in (3230,3200,3160):
        clipped=raw.copy();clipped[:,[7,11,15]]=np.minimum(clipped[:,[7,11,15]],cap)
        goals=clipped[:,ORDER]*TICK-np.pi
        for i,r in enumerate(resets):
            d=mujoco.MjData(m);d.qpos[:]=r['qpos'];d.ctrl[:]=goals[0];mujoco.mj_forward(m,d)
            start=d.qpos[-7:].copy();hold=longest=0;maxerr=pen=0.
            for goal in goals:
                d.ctrl[:]=goal
                for _ in range(round(.05/m.opt.timestep)):
                    mujoco.mj_step(m,d)
                    pen=max(pen,max([-float(c.dist)*1000 for c in d.contact if c.geom1 in support or c.geom2 in support]or[0]))
                maxerr=max(maxerr,float(np.max(np.abs(goal-d.qpos[:16]))))
                tip,axis,err=map(float,measure(start[-4:],d.qpos[-4:],xp=np));shift=float(np.linalg.norm(d.qpos[-7:-4]-start[:3]))
                hold=hold+1 if eligible(tip,axis,err,shift,xp=np) and pen<=.5 else 0;longest=max(longest,hold)
            row=dict(cap_ticks=cap,reset_index=i,longest_hold_steps=longest,max_tracking_rad=maxerr,support_penetration_mm=pen,
                     passed=bool(longest>=23 and maxerr<=.18 and pen<=.5 and np.isfinite(d.qpos).all()),final_target_error_deg=float(np.degrees(err)))
            rows.append(row);print(json.dumps(row),flush=True)
    (ROOT/'sac_distal_range_probe_result.json').write_text(json.dumps(dict(status='offline_range_probe',results=rows,
        motor_commands_sent=0,hardware_authorized=False,caveat='Goal caps are conservative candidates, not measured hardware limits; no live change.'),indent=2)+'\n')

if __name__=='__main__':main()
