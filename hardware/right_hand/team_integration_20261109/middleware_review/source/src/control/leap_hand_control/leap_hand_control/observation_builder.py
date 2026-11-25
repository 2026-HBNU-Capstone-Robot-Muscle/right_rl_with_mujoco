"""Build the reference LEAP policy's 108-value observation from ROS inputs."""

import numpy as np


JOINT_NAMES = (
    'if_mcp', 'if_rot', 'if_pip', 'if_dip',
    'mf_mcp', 'mf_rot', 'mf_pip', 'mf_dip',
    'rf_mcp', 'rf_rot', 'rf_pip', 'rf_dip',
    'th_cmc', 'th_axl', 'th_mcp', 'th_ipl',
)
HISTORY_LEN = 3
NQ = 16


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=np.float64)


def quat_inv(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64) / float(q @ q)


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),
         2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
         2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x),
         1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


class LeapObservationBuilder:
    """Maintain the same three-sample histories as the reference deployment."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.qpos_err_hist = np.zeros(HISTORY_LEN * NQ)
        self.cube_pos_err_hist = np.zeros(HISTORY_LEN * 3)
        self.cube_ori_err_hist = np.zeros(HISTORY_LEN * 6)
        self.last_act = np.zeros(NQ)

    def fill_orientation_history(self, cube_quat_wxyz, goal_quat_wxyz):
        """Seed all three orientation-history slots from an explicit plan state."""
        cube_quat = np.asarray(cube_quat_wxyz, dtype=np.float64)
        goal_quat = np.asarray(goal_quat_wxyz, dtype=np.float64)
        if (cube_quat.shape != (4,) or goal_quat.shape != (4,)
                or not np.all(np.isfinite(cube_quat))
                or not np.all(np.isfinite(goal_quat))):
            raise ValueError('Cube and goal orientations must be finite quaternions.')
        error = self._xmat_diff(cube_quat, goal_quat)
        self.cube_ori_err_hist = np.tile(error, HISTORY_LEN)

    @staticmethod
    def _xmat_diff(cube_quat, goal_quat):
        return quat_to_mat(quat_mul(cube_quat, quat_inv(goal_quat))).ravel()[3:]

    def build(self, q_meas, q_cmd, palm_position, cube_position,
              cube_quat_wxyz, goal_quat_wxyz):
        q_meas = np.asarray(q_meas, dtype=np.float64)
        q_cmd = np.asarray(q_cmd, dtype=np.float64)
        qe = q_meas - q_cmd
        self.qpos_err_hist = np.roll(self.qpos_err_hist, NQ)
        self.qpos_err_hist[:NQ] = qe

        position_error = (
            np.asarray(palm_position, dtype=np.float64)
            - np.asarray(cube_position, dtype=np.float64))
        self.cube_pos_err_hist = np.roll(self.cube_pos_err_hist, 3)
        self.cube_pos_err_hist[:3] = position_error

        qd = quat_mul(cube_quat_wxyz, quat_inv(goal_quat_wxyz))
        self.cube_ori_err_hist = np.roll(self.cube_ori_err_hist, 6)
        self.cube_ori_err_hist[:6] = quat_to_mat(qd).ravel()[3:]

        qn = qd / np.linalg.norm(qd)
        orientation_error = 2.0 * np.arcsin(min(np.linalg.norm(qn[1:]), 1.0))
        observation = np.concatenate([
            q_meas,
            self.qpos_err_hist,
            self.cube_pos_err_hist,
            self.cube_ori_err_hist,
            self.last_act,
            [orientation_error],
        ])
        if observation.shape != (108,) or not np.all(np.isfinite(observation)):
            raise ValueError('LEAP observation must be 108 finite values.')
        return observation

    def set_last_action(self, action):
        action = np.asarray(action, dtype=np.float64)
        if action.shape != (NQ,) or not np.all(np.isfinite(action)):
            raise ValueError('LEAP action must be 16 finite values.')
        self.last_act = action.copy()
