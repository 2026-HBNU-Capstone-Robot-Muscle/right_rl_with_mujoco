"""One-shot 100mA ID13 trial. No automatic escalation, no full-hand changes.

The 80mA trial saturated with ~5V feedback and matching base-joint settings.
This tests a higher current cap; it does NOT establish that friction is absent.
"""
from hand13_start_target import make_target, MAX_MOVE, MAX_TIME
import hand13_motion_diag as diag


def main():
    return diag.main(planner=make_target,max_move=MAX_MOVE,max_time=MAX_TIME,
                     current_ma=100,current_stop_ma=110,
                     confirmation='TARGET13_100',result_stem='hand13_start_target_100',
                     trial_note='ONE-SHOT ID13 80 -> 100mA TRIAL: same raw2570 target. No further automatic increase. Do not change supply voltage.')


if __name__=='__main__': raise SystemExit(main())
