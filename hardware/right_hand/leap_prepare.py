"""Empty-hand preparation verification before running contact or policy trials."""
import hand_prepare_full as motion
from held_policy_shadow import main

if __name__ == '__main__':
    print('LEAP prepare: Full XC330 550mA; smooth preparation -> hold 3s -> OFF.',flush=True)
    print('PID: preserve existing device gains (no example-gain override).',flush=True)
    print('EMPTY hand only. SPACE starts; Q/Esc/Ctrl+C stops. No policy goals sent.',flush=True)
    raise SystemExit(main(motion_runner=motion,confirmation=None,result_stem='leap_prepare',prepare_only=True))
