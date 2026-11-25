"""Offline ±5 mm cube-pose contact search around the measured October 10 start."""

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
    samples = [s for s in trial['samples'] if s['phase'] == 'place_cube']
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    set_cube_physics(model, mass_kg=.100)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(samples[-1]['actual']) * 2*np.pi/4096)
    q[-4:] = reg['cube_quat_world_candidate']
    center = np.asarray(reg['cube_center_world_m'])
    rows = []
    for dx in (-5., -2.5, 0., 2.5, 5.):
        for dy in (-5., -2.5, 0., 2.5, 5.):
            for dz in (-5., -2.5, 0., 2.5, 5.):
                q[-7:-4] = center + np.array([dx, dy, dz])/1000
                rows.append({'offset_mm': [dx, dy, dz], **probe(model, q, cube_geom)})
    plausible = [r for r in rows if r['penetration_mm'] <= 2 and
                 r['drift_mm'] <= 10 and r['initial_geoms'] and r['final_geoms']]
    report = {'status': 'offline_local_contact_diagnostic', 'hardware_authorized': False,
              'start_center_m': center.tolist(), 'sample_count': len(rows),
              'plausible_contact_count': len(plausible),
              'lowest_drift': sorted(rows, key=lambda r: r['drift_mm'])[:10],
              'plausible': sorted(plausible, key=lambda r: r['drift_mm'])[:10],
              'caveat': 'A local position search cannot calibrate missing collision geometry or prove real-world SAC readiness.'}
    (ROOT/'sac_local_contact_search_1010_result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
