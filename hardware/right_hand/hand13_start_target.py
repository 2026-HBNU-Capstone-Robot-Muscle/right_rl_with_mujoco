"""ID13-only diagnostic to the SAME DEFAULT_POSE target as the full-hand trial.

No policy. Other IDs stay OFF. This does not prove why full-hand motion stalled.
Use the shared tested 80mA safeguards without changing other test entrypoints.
"""
import numpy as np
import hand13_motion_diag as diag

TARGET_RAW = 2570
MAX_MOVE = .45
MAX_TIME = 25


def make_target(raw):
    default_goal = diag.target_plan(raw)
    if int(default_goal[diag.INDEX]) != TARGET_RAW:
        raise ValueError('Configured ID13 start target changed; no automatic adaptation')
    start = int(raw[diag.INDEX])
    if start >= TARGET_RAW:
        raise ValueError('ID13 already at/above raw2570; no reverse or repeated test')
    budget_ticks = int(np.floor(MAX_MOVE/diag.TICK))
    if TARGET_RAW-start+diag.ENABLE_MAX_SHIFT_TICKS > budget_ticks:
        raise ValueError('Raw2570 exceeds 0.45rad travel budget including enable margin; no movement')
    return TARGET_RAW


def main():
    return diag.main(planner=make_target,max_move=MAX_MOVE,max_time=MAX_TIME,
                     confirmation='TARGET13_80',result_stem='hand13_start_target',
                     trial_note='ID13 ISOLATED START TARGET: raw2570, same as full-hand test; prior plateau near raw2457. No automatic retries.')


if __name__ == '__main__': raise SystemExit(main())
