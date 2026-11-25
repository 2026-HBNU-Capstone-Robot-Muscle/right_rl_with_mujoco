"""Offline check whether one plausible gravity tilt stabilizes multiple real starts."""

import json
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_cube_model import set_cube_physics
from sac_index_middle_support_probe import probe
from sac_marker_plane_support_probe import RUNS

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent/'work/Leap_Test/deploy'))
from leap_hw import hw_to_sim


def main():
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    rows = []
    for label, reg_file, trial_file in RUNS:
        reg = json.loads((ROOT/reg_file).read_text())
        trial = json.loads((ROOT/trial_file).read_text())
        actual = [s for s in trial['samples'] if s['phase']=='place_cube'][-1]['actual']
        q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
        q[:16] = hw_to_sim(np.asarray(actual)*2*np.pi/4096)
        q[-7:-4] = reg['cube_center_world_m']
        q[-4:] = reg['cube_quat_world_candidate']
        for x_deg, y_deg in ((0.,0.),(10.,10.),(10.,20.),(20.,20.)):
            x,y = np.radians([x_deg,y_deg])
            Rx = np.array([[1,0,0],[0,np.cos(x),-np.sin(x)],[0,np.sin(x),np.cos(x)]])
            Ry = np.array([[np.cos(y),0,np.sin(y)],[0,1,0],[-np.sin(y),0,np.cos(y)]])
            model.opt.gravity[:] = Ry @ Rx @ np.array([0.,0.,-9.81])
            rows.append({'run':label,'x_tilt_deg':x_deg,'y_tilt_deg':y_deg,
                         **probe(model,q,cube_id)})
    report={'status':'offline_gravity_crossrun_sensitivity','hardware_authorized':False,
            'results':rows,
            'caveat':'The physical hand/table orientation is not measured; a stable simulated tilt is not a calibrated contact model.'}
    (ROOT/'sac_gravity_crossrun_probe_result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'status':report['status'],
                      'summary':[{'run':r['run'],'x':r['x_tilt_deg'],'y':r['y_tilt_deg'],
                                  'drift_mm':r['drift_mm']} for r in rows]},indent=2))


if __name__=='__main__':
    main()
