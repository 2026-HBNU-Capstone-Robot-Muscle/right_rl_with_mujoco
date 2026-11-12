"""Second loaded trial: 5s, +/-65 ticks, max 32 ticks per command."""
import hand_prepare_full as motion
from held_policy_shadow import main as run_main


def main():
    print('LEAP loaded policy 0.10: EMPTY prepare -> PLACE/SPACE -> POLICY 5s -> REMOVE/SPACE -> OFF.',flush=True)
    print('Range +/-0.10rad (65 ticks); each command max 32 ticks. All IDs 550mA; preserve device PID.',flush=True)
    print('Use padded catch area. Hands clear before SPACE; Q/faults stop. No automatic retry.',flush=True)
    return run_main(motion_runner=motion,confirmation=None,result_stem='leap_loaded_policy_010',
                    loaded_policy_mode=True,policy_radius_ticks=65)


if __name__=='__main__': raise SystemExit(main())
