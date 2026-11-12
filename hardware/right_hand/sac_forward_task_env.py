"""Experimental first-flip SAC task: reward the requested signed roll, not face-up alone.

This wraps an offline MuJoCo environment. It deliberately does not import or
write to the live controller. The contact/reset model is not yet validated.
"""

import jax.numpy as jp
from sac_forward_geometry import measure, eligible, reward_value, target_quaternion, METRIC_VERSION


GREEN = jp.asarray([0., -1., 0.])
ORANGE = jp.asarray([-1., 0., 0.])


def rotate(q, v):
    q = q / jp.linalg.norm(q)
    xyz = q[1:]
    return v + 2 * jp.cross(xyz, jp.cross(xyz, v) + q[0] * v)


def direction_components(start_quat, cube_quat):
    """Return signed twist and off-axis error; second value is NOT Euler yaw."""
    tip, off_axis, _ = measure(start_quat, cube_quat, xp=jp)
    return tip, off_axis


class ForwardFirstFlip:
    def __init__(self, env, direction_sign=1):
        if direction_sign not in (-1, 1):
            raise ValueError('direction_sign must be -1 or +1')
        self.env = env
        self.direction_sign = direction_sign
        if env._config.chain_goals or env._config.face_onehot_obs:
            raise ValueError('Directional task requires chain_goals=False and face_onehot_obs=False')

    def __getattr__(self, name):
        return getattr(self.env, name)

    def reset(self, rng):
        return self._initialize(self.env.reset(rng))

    def reset_at(self, rng, index):
        return self._initialize(self.env.reset_at(rng, index))

    def _initialize(self, state):
        info = dict(state.info)
        info['forward_start_quat'] = state.data.qpos[-4:]
        info['forward_start_center'] = state.data.qpos[-7:-4]
        info['forward_goal_hold'] = jp.zeros((), dtype=jp.int32)
        info['forward_previous_target_error'] = jp.asarray(jp.pi/2)
        goal = target_quaternion(info['forward_start_quat'], self.direction_sign, jp)
        data = state.data.replace(mocap_quat=jp.broadcast_to(goal, state.data.mocap_quat.shape))
        # The observation must describe the same full orientation as the reward.
        # Clear history from the base environment's unrelated face-only reset.
        for key in ('qpos_error_history', 'cube_pos_error_history', 'cube_ori_error_history'):
            info[key] = jp.zeros_like(info[key])
        for _ in range(self.env._config.history_len):
            obs = self.env._get_obs(data, info)
        metrics = dict(state.metrics)
        for name in ('forward/tip_deg', 'forward/off_axis_deg', 'forward/target_error_deg',
                     'forward/center_shift_m', 'forward/success'):
            metrics[name] = jp.zeros(())
        return state.replace(data=data, obs=obs, info=info, metrics=metrics)

    def step(self, state, action):
        stepped = self.env.step(state, action)
        tip, off_axis, target_error = measure(state.info['forward_start_quat'],
                                              stepped.data.qpos[-4:], self.direction_sign, jp)
        desired_tip = self.direction_sign * tip
        center_shift = jp.linalg.norm(stepped.data.qpos[-7:-4] -
                                      state.info['forward_start_center'])
        candidate = eligible(tip, off_axis, target_error, center_shift, self.direction_sign, jp)
        previous_hold = state.info['forward_goal_hold']
        if 'steps' in state.info:
            previous_hold = jp.where(state.info['steps'] == 0, 0, previous_hold)
        hold = jp.where(candidate, previous_hold + 1, 0)
        success = hold >= 3
        dropped = stepped.data.qpos[-5] < -.05
        invalid = (~jp.all(jp.isfinite(stepped.data.qpos)) |
                   ~jp.all(jp.isfinite(stepped.data.qvel)))
        # Signed progress makes the opposite roll actively undesirable. This
        # replaces, rather than adds to, the original direction-blind reward.
        reward = reward_value(target_error, off_axis, center_shift, action, success, dropped, jp,
                              state.info['forward_previous_target_error'])
        info = dict(stepped.info)
        info['forward_start_quat'] = state.info['forward_start_quat']
        info['forward_start_center'] = state.info['forward_start_center']
        info['forward_goal_hold'] = hold
        info['forward_previous_target_error'] = target_error
        metrics = dict(stepped.metrics)
        metrics['forward/tip_deg'] = jp.rad2deg(desired_tip)
        metrics['forward/off_axis_deg'] = jp.rad2deg(off_axis)
        metrics['forward/target_error_deg'] = jp.rad2deg(target_error)
        metrics['forward/center_shift_m'] = center_shift
        metrics['forward/success'] = success.astype(float)
        metrics['reward/success'] = success.astype(float)
        return stepped.replace(info=info, metrics=metrics, reward=reward,
                               done=(success | dropped | invalid).astype(reward.dtype))
