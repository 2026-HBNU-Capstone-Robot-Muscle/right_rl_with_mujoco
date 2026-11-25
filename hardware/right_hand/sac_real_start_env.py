"""Offline-only real-start wrapper for the installed LeapCubeFaceUp environment.

It changes only this process's MuJoCo model and reset distribution. It must
not be used to authorize live motor control: camera occlusion remains unsolved.
"""

import json
import sys
from pathlib import Path

import jax
import jax.numpy as jp
import numpy as np
from mujoco import mjx

from sac_cube_model import set_cube_physics

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


class RealStartFaceUp:
    def __init__(self, env):
        self.env = env
        trial = json.loads((ROOT / 'leap_support_capture_result.json').read_text())
        reg = json.loads((ROOT / 'sac_cube_rim_register_1010_stage0_result.json').read_text())
        tilt = json.loads((ROOT / 'table_tilt_20261010_053815_726640' /
                           'tilt_result.json').read_text())
        search = json.loads((ROOT / 'sac_measured_gravity_local_search_result.json').read_text())
        actual = [s for s in trial['samples'] if s['phase'] == 'place_cube'][-1]['actual']
        self.joints = jp.asarray(hw_to_sim(np.asarray(actual) * 2 * np.pi / 4096))
        self.cube_center = jp.asarray(reg['cube_center_world_m'])
        self.cube_quat = jp.asarray(reg['cube_quat_world_candidate'])
        offsets = [r['offset_mm'] for r in search['plausible']]
        if not offsets:
            raise RuntimeError('No measured-gravity contact candidates')
        self.offsets_m = jp.asarray(np.asarray(offsets) / 1000)
        model = env.mj_model
        set_cube_physics(model, mass_kg=.100)
        model.opt.gravity[:] = tilt['gravity_in_sim_world_m_s2_candidate']
        env._mjx_model = mjx.put_model(model, impl='jax')
        env._cube_mass = model.body_subtreemass[model.body('cube').id]

    def __getattr__(self, name):
        return getattr(self.env, name)

    def reset(self, rng):
        rng, pick_rng = jax.random.split(rng)
        state = self.env.reset(rng)
        index = jax.random.randint(pick_rng, (), 0, self.offsets_m.shape[0])
        qpos = state.data.qpos.at[:16].set(self.joints)
        qpos = qpos.at[-7:-4].set(self.cube_center + self.offsets_m[index])
        qpos = qpos.at[-4:].set(self.cube_quat)
        data = state.data.replace(qpos=qpos, qvel=jp.zeros_like(state.data.qvel),
                                  ctrl=self.joints)
        data = mjx.forward(self.env.mjx_model, data)
        info = dict(state.info)
        info['motor_targets'] = data.ctrl
        info['start_cube_pos'] = data.qpos[-7:-4]
        for key in ('qpos_error_history', 'cube_pos_error_history', 'cube_ori_error_history'):
            info[key] = jp.zeros_like(info[key])
        error = self.env._cube_orientation_error(data, info)
        info['prev_ori_error'] = error
        info['best_ori_error'] = error
        obs = self.env._get_obs(data, info)
        return state.replace(data=data, info=info, obs=obs)

    def step(self, state, action):
        return self.env.step(state, action)
