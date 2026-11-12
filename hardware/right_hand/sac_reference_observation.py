"""Pure NumPy 110-value observation adapter for the exported reference SAC.

No device access. Positions/quaternions must already share the simulation world
frame. This module cannot certify camera registration or invent occluded poses.
Call once per fresh control sample. Warm histories with three real samples before
using the actor; keep reference_step zero during that warmup. Advance reference
step only with a completed control step, not camera frames or wall-clock pauses.
"""
import numpy as np
from sac_reference_numpy import vector
from sac_forward_geometry import multiply

MOTOR_IDS_IN_SIM_ORDER = np.r_[np.arange(5, 17), np.arange(1, 5)]


def motor_ticks_to_sim(ticks):
    ticks = vector(ticks, 16, 'motor ticks in ID 1..16 order')
    return (ticks[MOTOR_IDS_IN_SIM_ORDER - 1] * (2*np.pi/4096) - np.pi).astype(np.float32)


def unit_quaternion(value, name):
    q = vector(value, 4, name)
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        raise ValueError(name + ': zero quaternion')
    return q / norm


def rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
    ], dtype=np.float32)


class ReferenceObservation:
    def __init__(self):
        self.reset()

    def reset(self):
        self.joint_history = np.zeros((3, 16), dtype=np.float32)
        self.position_history = np.zeros((3, 3), dtype=np.float32)
        self.orientation_history = np.zeros((3, 6), dtype=np.float32)
        self.samples = 0

    @property
    def warmed(self):
        return self.samples >= 3

    def build(self, measured_joints, commanded_joints, palm_position, cube_position,
              cube_quaternion, goal_quaternion, last_executed_action, reference_step):
        """Angles in sim order/radians, positions metres, quaternions wxyz.

        commanded_joints is the last sent goal (not measured angles).
        last_executed_action is ReferencePolicy.command()[1], not actor residual.
        A changed goal/frame requires reset and fresh history warmup.
        """
        # Validate everything before mutating histories.
        qm = vector(measured_joints, 16, 'measured joints')
        qc = vector(commanded_joints, 16, 'commanded joints')
        palm = vector(palm_position, 3, 'palm position')
        cube = vector(cube_position, 3, 'cube position')
        q = unit_quaternion(cube_quaternion, 'cube quaternion')
        goal = unit_quaternion(goal_quaternion, 'goal quaternion')
        action = vector(last_executed_action, 16, 'last executed action')
        if np.any(abs(action) > 1.000001):
            raise ValueError('Executed action outside [-1, 1]')
        if isinstance(reference_step, (bool, np.bool_)) or not isinstance(reference_step, (int, np.integer)) or reference_step < 0:
            raise ValueError('reference_step must be a nonnegative integer')
        delta = unit_quaternion(multiply(q, goal * [1, -1, -1, -1]), 'quaternion difference')
        orientation = rotation_matrix(delta).ravel()[3:]
        for history, value in ((self.joint_history, qm-qc),
                               (self.position_history, palm-cube),
                               (self.orientation_history, orientation)):
            history[1:] = history[:-1].copy()
            history[0] = value
        self.samples += 1
        elapsed = np.float32(reference_step) * np.float32(.05)
        phase = [min(elapsed/np.float32(3.6), 1.), float(elapsed >= np.float32(3.6))]
        angle = 2*np.arcsin(np.clip(np.linalg.norm(delta[1:]), 0, 1))
        return np.concatenate([qm, self.joint_history.ravel(), self.position_history.ravel(),
                               self.orientation_history.ravel(), action, [angle], phase]).astype(np.float32)
