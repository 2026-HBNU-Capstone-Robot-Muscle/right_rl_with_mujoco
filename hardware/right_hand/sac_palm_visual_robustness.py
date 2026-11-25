"""Offline robustness check around one fitted, unvalidated palm-mesh start."""
import json
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry

from sac_palm_recompiled_collision_probe import make_model


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def trial(model, q):
    data = mujoco.MjData(model)
    data.qpos[:] = q
    data.ctrl[:] = q[:16]
    cube = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube')
    samples = []
    for step in range(101):
        if step in (0, 30, 100):
            mujoco.mj_forward(model, data)
            contact = [c for c in data.contact if cube in (c.geom1, c.geom2)]
            samples.append({'t_s': step * model.opt.timestep,
                            'contacts': len(contact),
                            'penetration_mm': -1000 * min([0.] + [float(c.dist) for c in contact]),
                            'drift_mm': float(1000 * np.linalg.norm(data.qpos[-7:-4] - q[-7:-4]))})
        if step < 100:
            data.qpos[:16] = q[:16]
            data.qvel[:16] = 0
            mujoco.mj_forward(model, data)
            mujoco.mj_step(model, data)
    return samples


def main():
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model, _ = make_model(env, 'visual_without_boxes')
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    trial_data = json.loads((ROOT / 'leap_support_capture_result.json').read_text(encoding='utf-8'))
    ticks = [r for r in trial_data['samples'] if r['phase'] == 'place_cube'][-1]['actual']
    q[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
    anchor = json.loads((ROOT / 'sac_registration_support_photo_diagnostic.json').read_text(encoding='utf-8'))
    base = np.asarray(anchor['cube_center_in_sim_world_candidate_m'], dtype=float)
    q[-4:] = anchor['cube_quat_world_candidate']
    rows = []
    for dx_mm in (12, 14, 16):
        for dz_mm in (-4, -2, 0):
            q[-7:-4] = base + np.array([dx_mm, 0, dz_mm]) / 1000
            rows.append({'dx_mm': dx_mm, 'dz_mm': dz_mm, 'samples': trial(model, q)})
    result = {'rows': rows, 'hardware_authorized': False,
              'caveat': 'Fitted start is from an unvalidated single-marker reset and not a physical calibration.'}
    (ROOT / 'sac_palm_visual_robustness_result.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(rows, indent=2))


if __name__ == '__main__':
    main()
