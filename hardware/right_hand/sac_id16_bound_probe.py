"""Offline hypothesis: ID16 cannot exceed the observed plateau. No hardware export."""
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from sac_supported_start_env import SupportedStart
from sac_forward_geometry import measure,eligible
ROOT=Path(__file__).resolve().parent
TICK=2*np.pi/4096

def main():
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=SupportedStart(registry.load('LeapCubeFaceUp',config=cfg),diagnostic_cube_side_mm=55)
    model=env.mj_model;support={model.geom(n).id for n in ('support_glass','support_wood')}
    doc=json.loads((ROOT/'sac_reachable_empty_path.json').read_text())
    raw=np.array([doc['start_ticks']]+doc['goals_id_order']);order=np.array(list(range(4,16))+list(range(4)))
    goals=raw[:,order]*TICK-np.pi;index=11;joint=int(model.actuator_trnid[index,0])
    if model.joint(joint).name!='rf_dip':raise RuntimeError('ID16 mapping differs')
    upper=3254*TICK-np.pi
    model.jnt_range[joint,1]=upper
    resets=json.loads((ROOT/'sac_cube55_resets.json').read_text())['candidates']
    rows=[]
    for cap in (3254,3238,3222):
        clipped=goals.copy();clipped[:,index]=np.minimum(clipped[:,index],cap*TICK-np.pi)
        for i,r in enumerate(resets):
            d=mujoco.MjData(model);d.qpos[:]=r['qpos'];d.qpos[index]=min(d.qpos[index],upper);d.ctrl[:]=clipped[0]
            mujoco.mj_forward(model,d);initial=d.qpos[-7:].copy();hold=longest=0;penetration=0.;error=0.
            for goal in clipped:
                d.ctrl[:]=goal
                for _ in range(round(.05/model.opt.timestep)):
                    mujoco.mj_step(model,d)
                    penetration=max(penetration,max([-float(c.dist)*1000 for c in d.contact if c.geom1 in support or c.geom2 in support]or[0]))
                error=max(error,float(np.max(np.abs(d.qpos[:16]-goal))))
                tip,axis,err=map(float,measure(initial[-4:],d.qpos[-4:],xp=np));shift=float(np.linalg.norm(d.qpos[-7:-4]-initial[:3]))
                hold=hold+1 if eligible(tip,axis,err,shift,xp=np) and penetration<=.5 else 0;longest=max(longest,hold)
            row=dict(cap_ticks=cap,reset_index=i,longest_hold_steps=longest,flip_success=longest>=23,
                     max_tracking_rad=error,support_penetration_mm=penetration,final_target_error_deg=float(np.degrees(err)))
            rows.append(row);print(json.dumps(row),flush=True)
    result=dict(status='hypothetical_mechanical_bound_probe',motor_commands_sent=0,hardware_authorized=False,
                observed_plateau_ticks=[3253,3254],assumed_id16_upper_ticks=3254,results=rows,
                caveat='Observed immobility does not prove a permanent joint limit. Physical contact and cabling must be inspected; no live path changed.')
    (ROOT/'sac_id16_bound_probe_result.json').write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':main()
