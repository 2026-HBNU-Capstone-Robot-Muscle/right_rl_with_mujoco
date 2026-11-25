"""Screen the five trained placements under the newly measured 55mm geometry."""
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_wood_glass_model import build
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent

def main():
    measurement=json.loads((ROOT/'cube_measurement_20261014.json').read_text())
    source=ROOT/'sac_supported_reset_result.json';resets=json.loads(source.read_text())
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    model,_=build(registry.load('LeapCubeFaceUp',config=cfg))
    physics=set_cube_physics(model,side_m=measurement['cube_side_mm']/1000,mass_kg=.100)
    model.actuator_ctrlrange[[0,4,8],0]=resets['root_command_floor_rad']
    cube=model.geom('cube').id;support={model.geom(n).id for n in ('support_glass','support_wood')}
    accepted=[];specs=[];rejected=[]
    for i,reset in enumerate(resets['candidates']):
        data=mujoco.MjData(model);data.qpos[:]=reset['qpos'];data.ctrl[:]=reset['ctrl'];mujoco.mj_forward(model,data)
        penetration=max([max(0,-float(c.dist)*1000) for c in data.contact if cube in (c.geom1,c.geom2)]or[0])
        spec=dict(source_offset_index=i,initial_cube_penetration_mm=penetration)
        if penetration>2:rejected.append(spec|dict(reason='initial_cube_overlap_above_2mm'));continue
        support_peak=0.
        for _ in range(round(.5/model.opt.timestep)):mujoco.mj_step(model,data)
        settled=data.qpos.copy();data.qvel[:]=0;drift=0.
        for _ in range(round(.5/model.opt.timestep)):
            mujoco.mj_step(model,data)
            drift=max(drift,float(np.linalg.norm(data.qpos[-7:-4]-settled[-7:-4])*1000))
            support_peak=max(support_peak,max([max(0,-float(c.dist)*1000) for c in data.contact if c.geom1 in support or c.geom2 in support]or[0]))
        spec.update(hold_drift_mm=drift,support_penetration_mm=support_peak,
                    cube_shift_from_training_mm=((settled[-7:-4]-np.array(reset['qpos'])[-7:-4])*1000).tolist())
        if drift>2 or support_peak>.5 or not np.isfinite(data.qpos).all():
            rejected.append(spec|dict(reason='unstable_or_support_contact'));continue
        accepted.append(dict(qpos=settled.tolist(),ctrl=reset['ctrl'],offset_mm=reset['offset_mm'],passed=True));specs.append(spec)
    result=dict(status='offline_cube_dimension_starts_screened',cube_side_mm=measurement['cube_side_mm'],
                physics=physics,candidates=accepted,specs=specs,rejected=rejected,motor_commands_sent=0,
                source_reset_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                caveat='Native settling only; transfer SAC evaluation and physical validation remain separate.')
    (ROOT/'sac_cube55_resets.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='candidates'},indent=2),flush=True)

if __name__=='__main__':main()
