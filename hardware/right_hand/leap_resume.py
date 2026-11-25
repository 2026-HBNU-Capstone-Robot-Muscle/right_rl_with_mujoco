"""One run: recovery -> controlled posture -> bounded policy motion.

The hand remains empty, the cube stays on a separate support, and policy
actions are rate/range limited. Recoverable tracking lag pauses target advance.
"""
import hand_prepare_full as motion
from held_policy_shadow import main


if __name__ == '__main__':
    print('LEAP resume: Full 550mA; recovering preparation speed; SPACE start; BOUNDED POLICY MOTION +/-0.05rad, 20Hz, 10s; empty hand.', flush=True)
    raise SystemExit(main(motion_runner=motion, confirmation=None,
                         result_stem='leap_resume', policy_mode=True))
