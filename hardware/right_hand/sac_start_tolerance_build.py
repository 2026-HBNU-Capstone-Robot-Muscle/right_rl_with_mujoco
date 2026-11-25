"""Screen off-nominal placements; preserve requested AND settled offsets."""
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent

def main():
    reset=json.loads((ROOT/'sac_supported_reset_result.json').read_text())['candidates'][0]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    model,_=build(registry.load('LeapCubeFaceUp',config=cfg))
    initial=np.asarray(reset['qpos']);ctrl=np.asarray(reset['ctrl'])
    d=mujoco.MjData(model);d.qpos[:]=initial;mujoco.mj_forward(model,d)
    rotation=d.xmat[model.body('palm').id].reshape(3,3).copy()
    candidates=[];specs=[];rejected=[]
    for x,y in ((0,0),(5,0),(-5,0),(0,5),(0,-5),(10,0),(-10,0),(0,10),(0,-10)):
        mujoco.mj_resetData(model,d);d.qpos[:]=initial;d.ctrl[:]=ctrl
        delta=rotation@np.array([x,y,0.])/1000
        d.qpos[-7:-4]+=delta;mujoco.mj_forward(model,d)
        cube_id=model.geom('cube').id
        penetration=max([max(0,-float(c.dist)*1000) for c in d.contact if cube_id in (c.geom1,c.geom2)]or[0])
        spec=dict(requested_palm_plane_offset_mm=[x,y,0],initial_cube_penetration_mm=penetration)
        if penetration>2:
            rejected.append(spec|dict(reason='cube_initial_penetration_above_2mm'));continue
        # Let gravity/contact seat the object; do not disguise displacement as
        # tolerance to the originally requested (possibly unsupported) position.
        for _ in range(round(.5/model.opt.timestep)):mujoco.mj_step(model,d)
        settled=d.qpos.copy();position=settled[-7:-4].copy();max_drift=0.
        d.qvel[:]=0
        for _ in range(round(.5/model.opt.timestep)):
            mujoco.mj_step(model,d)
            max_drift=max(max_drift,float(np.linalg.norm(d.qpos[-7:-4]-position)*1000))
        spec.update(settled_world_offset_mm=((position-initial[-7:-4])*1000).tolist(),
                    retained_palm_offset_mm=(rotation.T@(position-initial[-7:-4])*1000).tolist(),
                    hold_drift_mm=max_drift)
        if max_drift>2 or not np.isfinite(settled).all():
            rejected.append(spec|dict(reason='not_stationary_after_settling'));continue
        candidates.append(dict(qpos=settled.tolist(),ctrl=ctrl.tolist(),offset_mm=[x,y,0],passed=True))
        specs.append(spec)
    out=dict(status='offline_tolerance_starts_screened',candidates=candidates,specs=specs,rejected=rejected,
             motor_commands_sent=0,caveat='Requested planar offsets may settle elsewhere; only retained offsets characterize evaluation. No change to trained reset or hardware.')
    (ROOT/'sac_start_tolerance_resets.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({k:v for k,v in out.items() if k!='candidates'},indent=2),flush=True)

if __name__=='__main__':main()
