"""Offline contact sensitivity to collision friction at the measured first pose."""

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
sys.path.insert(0, str(ROOT.parent / 'work/Leap_Test/deploy'))
from leap_hw import hw_to_sim


def main():
    registration = json.loads((ROOT / 'sac_cube_rim_register_1010_stage0_result.json').read_text())
    trial = json.loads((ROOT / 'leap_support_capture_result.json').read_text())
    assert trial['all_torque_off_verified'] and trial['status'] == 'support_photo_complete'
    samples = [s for s in trial['samples'] if s['phase'] == 'place_cube']
    config = registry.get_default_config('LeapCubeFaceUp')
    config.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=config)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(samples[-1]['actual']) * 2*np.pi/4096)
    q[-7:-4] = registration['cube_center_world_m']
    q[-4:] = registration['cube_quat_world_candidate']
    original = model.geom_friction.copy()
    rows = []
    for scale in (1., 2., 4., 8.):
        model.geom_friction[:] = original
        model.geom_friction[:, 0] = original[:, 0] * scale
        rows.append({'sliding_friction_scale': scale,
                     'cube_sliding_friction': float(model.geom_friction[cube_geom, 0]),
                     **probe(model, q, cube_geom)})
    report = {'status': 'offline_friction_sensitivity', 'hardware_authorized': False,
              'registration': 'sac_cube_rim_register_1010_stage0_result.json',
              'trial': 'leap_support_capture_result.json', 'results': rows,
              'caveat': 'Friction scaling is diagnostic; no material coefficient was measured or changed on hardware.'}
    (ROOT/'sac_friction_probe_1010_result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
