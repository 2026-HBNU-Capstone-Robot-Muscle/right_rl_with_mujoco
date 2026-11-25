"""First bounded policy motion with the operator-placed cube on the hand."""
import hand_prepare_full as motion
from held_policy_shadow import main

if __name__=='__main__':
    print('LEAP loaded policy: EMPTY prepare -> PLACE/SPACE -> POLICY 5s -> REMOVE/SPACE -> OFF.',flush=True)
    print('Policy goals WILL move fingers within +/-0.05rad. 550mA; preserve device PID.',flush=True)
    print('Use padded catch area. Hands clear before SPACE; Q/faults turn torque OFF immediately.',flush=True)
    raise SystemExit(main(motion_runner=motion,confirmation=None,result_stem='leap_loaded_policy',loaded_policy_mode=True))
