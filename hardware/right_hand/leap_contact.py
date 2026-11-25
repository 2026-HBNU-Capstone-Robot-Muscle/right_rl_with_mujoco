"""Prepare empty hand, then operator-paced fixed-goal cube loading check."""
import hand_prepare_full as motion
from held_policy_shadow import main

if __name__ == '__main__':
    print('LEAP contact: EMPTY preparation -> PLACE -> fixed 5s observation -> REMOVE -> OFF.', flush=True)
    print('Full XC330: all IDs 550mA; measured stop above 650mA; preserve device PID gains.', flush=True)
    print('No combined timeout. PLACE/REMOVE wait for SPACE; motion faults and Q still stop.', flush=True)
    print('Use a padded catch area; faults/Q turn torque OFF immediately. Do not force cube between fingers.', flush=True)
    raise SystemExit(main(motion_runner=motion, confirmation=None,
                         result_stem='leap_contact', contact_mode=True))
