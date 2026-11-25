"""Offline test whether uncertain hand orientation relative to gravity explains drift."""

import json
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics
from sac_index_middle_support_probe import probe

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent/'work/Leap_Test/deploy'))
from leap_hw import hw_to_sim


def main():
    reg = json.loads((ROOT/'sac_cube_rim_register_1010_stage0_result.json').read_text())
    trial = json.loads((ROOT/'leap_support_capture_result.json').read_text())
    actual = [s for s in trial['samples'] if s['phase']=='place_cube'][-1]['actual']
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(actual)*2*np.pi/4096)
    q[-7:-4] = reg['cube_center_world_m']
    q[-4:] = reg['cube_quat_world_candidate']
    rows = []
    for x_deg in (-20.,-10.,0.,10.,20.):
        for y_deg in (-20.,-10.,0.,10.,20.):
            x,y = np.radians([x_deg,y_deg])
            Rx = np.array([[1,0,0],[0,np.cos(x),-np.sin(x)],[0,np.sin(x),np.cos(x)]])
            Ry = np.array([[np.cos(y),0,np.sin(y)],[0,1,0],[-np.sin(y),0,np.cos(y)]])
            model.opt.gravity[:] = Ry @ Rx @ np.array([0.,0.,-9.81])
            rows.append({'x_tilt_deg':x_deg,'y_tilt_deg':y_deg,**probe(model,q,cube_id)})
    report = {'status':'offline_gravity_orientation_sensitivity','hardware_authorized':False,
              'baseline':next(r for r in rows if r['x_tilt_deg']==0 and r['y_tilt_deg']==0),
              'stable_count':sum(r['drift_mm']<=10 and r['penetration_mm']<=2 for r in rows),
              'best':sorted(rows,key=lambda r:r['drift_mm'])[:8],
              'caveat':'Tilting gravity is a sensitivity analysis, not a measured hand mount orientation.'}
    (ROOT/'sac_gravity_tilt_probe_1010_result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
