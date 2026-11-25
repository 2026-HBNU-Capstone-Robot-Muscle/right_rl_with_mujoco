"""Explicitly armed ID13-only approach to the prior stall region. NOT SAC.

Fixed endpoint 2400 is just beyond the observed 2366-2373 plateau, not the
full DEFAULT_POSE target 2570. Current and tracking protections are unchanged.
"""
import numpy as np
import hand13_motion_diag as diag

TARGET_RAW = 2400
MAX_MOVE = .45
MAX_TIME = 25


def make_target(raw):
    # Reuse existing encoder, model-range and initial-envelope validation.
    default_goal = diag.target_plan(raw)
    start = int(raw[diag.INDEX])
    if start >= 2366:
        raise ValueError('ID13 already at/above prior stall entry (2366); no automatic reverse/retry')
    if not 0 < (TARGET_RAW-start)*diag.TICK <= MAX_MOVE:
        raise ValueError('Reaching raw2400 exceeds 0.45rad from current position; no movement')
    if TARGET_RAW > default_goal[diag.INDEX]:
        raise ValueError('Approach endpoint exceeds preparation target')
    # Reserve four ticks for a possible negative enable transition, so its
    # accepted hold cannot extend the movement budget or truncate the endpoint.
    if TARGET_RAW-start+diag.ENABLE_MAX_SHIFT_TICKS > int(np.floor(MAX_MOVE/diag.TICK)):
        raise ValueError('Insufficient travel margin for enable check; no movement')
    return TARGET_RAW


def main():
    return diag.main(planner=make_target,max_move=MAX_MOVE,max_time=MAX_TIME,
                     confirmation='APPROACH13_80',result_stem='hand13_stall_approach',
                     trial_note='PRIOR STALL APPROACH: fixed endpoint raw2400; prior plateau raw2366-2373. No automatic retries.')


if __name__ == '__main__': raise SystemExit(main())
