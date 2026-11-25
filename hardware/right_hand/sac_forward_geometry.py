"""Shared offline NumPy/JAX quaternion geometry for the first vertical flip.

Cube-local green normal is -Y and initial orange-front normal is -X.
The requested first flip is therefore +90 degrees about cube-local -Z.
No projected front vector is used: that vector vanishes at a quarter-turn.
"""
import numpy as np

METRIC_VERSION = 'forward_quaternion_v2'


def multiply(a, b, xp=np):
    return xp.concatenate((xp.asarray([a[0]*b[0] - xp.dot(a[1:], b[1:])]),
                           a[0]*b[1:] + b[0]*a[1:] + xp.cross(a[1:], b[1:])))


def normalized(q, xp=np):
    q = xp.asarray(q)
    return q / xp.maximum(xp.linalg.norm(q), 1e-12)


def target_quaternion(start, direction_sign=1, xp=np):
    half = xp.sqrt(xp.asarray(.5))
    turn = xp.stack((half, half*0, half*0, -direction_sign*half))
    return multiply(normalized(start, xp), turn, xp)


def measure(start, current, direction_sign=1, xp=np):
    start, current = normalized(start, xp), normalized(current, xp)
    relative = multiply(start * xp.asarray([1., -1., -1., -1.]), current, xp)
    relative = normalized(relative, xp)
    twist_norm = xp.linalg.norm(relative[xp.asarray([0, 3])])
    raw_twist = 2 * xp.arctan2(-relative[3], relative[0])
    tip = xp.where(twist_norm > 1e-7,
                   xp.arctan2(xp.sin(raw_twist), xp.cos(raw_twist)), 0.)
    # Swing is deviation from the intended rotation axis, not Euler yaw.
    off_axis = 2 * xp.arctan2(xp.linalg.norm(relative[1:3]), twist_norm)
    target = target_quaternion(start, direction_sign, xp)
    delta = multiply(target * xp.asarray([1., -1., -1., -1.]), current, xp)
    target_error = 2 * xp.arctan2(xp.linalg.norm(delta[1:]), xp.abs(delta[0]))
    return tip, off_axis, target_error


def eligible(tip, off_axis, target_error, center_shift, direction_sign=1, xp=np):
    tip = direction_sign * tip
    return ((tip >= xp.deg2rad(70.)) & (tip <= xp.deg2rad(110.)) &
            (off_axis <= xp.deg2rad(25.)) & (target_error <= xp.deg2rad(25.)) &
            (center_shift <= .030))


def reward_value(target_error, off_axis, center_shift, action, success, dropped,
                 xp=np, previous_target_error=None):
    previous = xp.pi/2 if previous_target_error is None else previous_target_error
    # Progress telescopes over a trajectory. Holding just outside success must
    # not earn positive reward forever in preference to completing the flip.
    return (8 * (previous-target_error) - .05 - .2*target_error
            - .5 * off_axis / (xp.pi/2)
            - 20 * xp.maximum(center_shift - .015, 0.)
            - .005 * xp.sum(xp.square(action))
            + 20 * xp.asarray(success, dtype=float) - 20 * xp.asarray(dropped, dtype=float))
