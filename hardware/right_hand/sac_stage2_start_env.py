"""Experimental October-11 stage-2 reset for directional SAC, offline only.

This reset is fitted to one physical trial's failed left turn. It is not a
validated sim-to-real contact model or authority to export a motor policy.
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


class Stage2StartFaceUp:
    def __init__(self, env, friction_scale=2.0):
        self.env = env
        run = json.loads((ROOT / 'leap_sequence_flip_20261011_051518_433595.json').read_text())
        stage2 = [row for row in run['samples'] if row['phase'] == 'stage_2'][-1]
        reg = json.loads((ROOT / 'sac_stage2_anchor_registration_result.json').read_text())
        tilt = json.loads((ROOT / 'table_tilt_20261010_053815_726640' /
                           'tilt_result.json').read_text())
        self.joints = jp.asarray(hw_to_sim(
            np.asarray(stage2['actual']) * 2 * np.pi / 4096))
        self.cube_center = jp.asarray(reg['cube_center_world_m'])
        self.cube_quat = jp.asarray(reg['cube_quat_world_candidate'])
        # Include both sides of the observed 0.625-mm contact-regime edge;
        # this is a stress test, not a calibrated physical placement range.
        self.offsets_m = jp.asarray(np.asarray([
            [1.25, 0, 0], [1.875, 0, 0], [2.5, 0, 0],
            [1.875, -1.5, 0], [1.875, 1.5, 0],
            [1.875, 0, -1.5], [1.875, 0, 1.5],
        ]) / 1000)
        model = env.mj_model
        set_cube_physics(model, mass_kg=.100)
        model.geom_friction[:, 0] *= friction_scale
        model.opt.gravity[:] = tilt['gravity_in_sim_world_m_s2_candidate']
        env._mjx_model = mjx.put_model(model, impl='jax')
        env._cube_mass = model.body_subtreemass[model.body('cube').id]
        self.friction_scale = friction_scale

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
        for key in ('qpos_error_history', 'cube_pos_error_history',
                    'cube_ori_error_history'):
            info[key] = jp.zeros_like(info[key])
        error = self.env._cube_orientation_error(data, info)
        info['prev_ori_error'] = error
        info['best_ori_error'] = error
        obs = self.env._get_obs(data, info)
        return state.replace(data=data, obs=obs, info=info)

    def step(self, state, action):
        return self.env.step(state, action)
