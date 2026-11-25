"""Explicit full-hand trial: ID13 100mA, all other IDs 60mA. NOT SAC."""
import hand_start_pose as base


def main():
    return base.main(id13_current_ma=100, confirmation='START_V3',
                     result_stem='hand_start_pose_v3', version='v3')


if __name__ == '__main__':
    raise SystemExit(main())
