"""Inspect the offline model's ID12 joint limit against archived encoder readings."""

import json
import sys
from pathlib import Path

import mujoco
from mujoco_playground import registry
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'deploy'))
from leap_hw import hw_to_sim


def main():
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'mf_dip')
    run = json.loads((ROOT / 'leap_sequence_flip_20261011_051518_433595.json').read_text())
    rows = [r for r in run['samples'] if r['phase'] in ('stage_4', 'stage_5')]
    last = rows[-1]
    goal = hw_to_sim(np.asarray(last['goal']) * (2 * np.pi / 4096))[7]
    actual = hw_to_sim(np.asarray(last['actual']) * (2 * np.pi / 4096))[7]
    report = dict(status='offline_ID12_joint_limit_comparison', motor_commands_sent=0,
                  motor_id=12, model_joint='mf_dip',
                  model_joint_range_rad=model.jnt_range[joint_id].tolist(),
                  model_joint_limited=bool(model.jnt_limited[joint_id]),
                  stage_5_goal_rad=float(goal), stage_5_actual_rad=float(actual),
                  stage_5_goal_tick=int(last['goal'][11]),
                  stage_5_actual_tick=int(last['actual'][11]),
                  caveat='The observed encoder plateau is not a measured hard-stop angle. The joint range alone cannot identify colliding links or the exact mechanical stop.')
    (ROOT / 'sac_joint_limit_check_result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
