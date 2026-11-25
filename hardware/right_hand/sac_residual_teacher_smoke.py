"""Offline parity and one-step smoke test for the residual SAC teacher."""

import json
import sys
from pathlib import Path

import jax
import jax.numpy as jp
import numpy as np
from mujoco_playground import registry

from sac_real_start_env import RealStartFaceUp
from sac_residual_real_start_env import JaxTeacher, POLICY, ResidualFaceUp

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from policy_np import Policy


def main():
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = ResidualFaceUp(RealStartFaceUp(registry.load('LeapCubeFaceUp', config=cfg)))
    state = jax.jit(env.reset)(jax.random.PRNGKey(0))
    obs = np.asarray(state.obs['state'])
    teacher_np = Policy(POLICY)(obs)
    teacher_jax = np.asarray(jax.jit(env.teacher)(jp.asarray(obs)))
    difference = float(np.max(np.abs(teacher_np - teacher_jax)))
    assert difference < 1e-4, difference
    next_state = jax.jit(env.step)(state, jp.zeros(16))
    assert np.isfinite(np.asarray(next_state.obs['state'])).all()
    result = {'status': 'offline_residual_teacher_smoke',
              'hardware_authorized': False,
              'max_action_difference_from_numpy': difference,
              'one_step_done': bool(next_state.done),
              'cube_drift_mm': float(1000 * np.linalg.norm(
                  np.asarray(next_state.data.qpos[-7:-4]) -
                  np.asarray(state.data.qpos[-7:-4])))}
    (ROOT / 'sac_residual_teacher_smoke_result.json').write_text(
        json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
