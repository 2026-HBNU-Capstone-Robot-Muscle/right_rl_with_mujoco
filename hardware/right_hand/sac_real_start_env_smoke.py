"""Check the offline real-start reset and one simulated action; no device I/O."""

import json
from pathlib import Path

import jax
import jax.numpy as jp
import numpy as np
from mujoco_playground import registry

from sac_real_start_env import RealStartFaceUp

ROOT = Path(__file__).resolve().parent


def main():
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = RealStartFaceUp(registry.load('LeapCubeFaceUp', config=cfg))
    reset = jax.jit(env.reset)
    step = jax.jit(env.step)
    rows = []
    for seed in (0, 1, 2, 3):
        state = reset(jax.random.PRNGKey(seed))
        center = np.asarray(state.data.qpos[-7:-4])
        before = np.asarray(state.obs['state'])
        assert before.shape == (108,) and np.isfinite(before).all()
        after = step(state, jp.zeros(env.action_size))
        after_obs = np.asarray(after.obs['state'])
        assert np.isfinite(after_obs).all()
        rows.append({'seed': seed, 'cube_center_m': center.tolist(),
                     'one_step_drift_mm': float(1000 * np.linalg.norm(
                         np.asarray(after.data.qpos[-7:-4]) - center)),
                     'one_step_done': bool(after.done),
                     'observation_shape': list(before.shape)})
    result = {'status': 'offline_real_start_env_smoke', 'hardware_authorized': False,
              'cube_side_m': .0545, 'cube_mass_kg': .100,
              'gravity_sim_m_s2': np.asarray(env.mj_model.opt.gravity).tolist(),
              'contact_candidate_count': int(env.offsets_m.shape[0]),
              'results': rows,
              'caveat': 'Only four simulated resets and one action each. This does not establish learnability or real-hand control.'}
    (ROOT / 'sac_real_start_env_smoke_result.json').write_text(
        json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
