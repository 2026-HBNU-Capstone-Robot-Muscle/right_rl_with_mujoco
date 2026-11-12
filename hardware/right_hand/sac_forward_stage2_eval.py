"""Independent deterministic rollout of the offline directional SAC pilot."""

import argparse
import json
from pathlib import Path

import jax
import jax.numpy as jp
import numpy as np
from brax.training.agents.sac import checkpoint
from mujoco_playground import registry

from sac_forward_flip_gate import matrix
from sac_forward_task_env import ForwardFirstFlip
from sac_ordered_cube_goal import COLORS, NORMALS
from sac_stage2_start_env import Stage2StartFaceUp

ROOT = Path(__file__).resolve().parent


def face_toward(R, direction):
    return int(np.argmax((R @ NORMALS.T).T @ direction))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', default='sac_forward_stage2_pilot_2k_v2')
    parser.add_argument('--seeds', nargs='+', type=int, default=[0, 1, 2, 3, 4, 5, 6])
    parser.add_argument('--max-steps', type=int, default=180)
    args = parser.parse_args()
    raise RuntimeError('This evaluator describes the archived v1 reward. That reward had a 90-degree singularity. Use sac_forward_eval.py for newly trained stage-0 v2 runs; old reports remain archived and are not overwritten.')
    checkpoint_paths = sorted((ROOT / args.run / 'checkpoints').glob('[0-9]*'))
    if not checkpoint_paths:
        raise RuntimeError('No checkpoint')
    actor = jax.jit(checkpoint.load_policy(str(checkpoint_paths[-1]), deterministic=True))
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    cfg.target_face = 2
    env = ForwardFirstFlip(Stage2StartFaceUp(registry.load('LeapCubeFaceUp', config=cfg)))
    reset_fn = jax.jit(env.reset)
    step_fn = jax.jit(env.step)
    rows = []
    for seed in args.seeds:
        state = reset_fn(jax.random.PRNGKey(seed))
        start_q = np.asarray(state.data.qpos[-4:])
        R0 = matrix(start_q)
        initial_up = R0 @ NORMALS[5]
        initial_front = R0 @ NORMALS[3]
        offset_mm = 1000 * (np.asarray(state.data.qpos[-7:-4]) -
                            np.asarray(env.cube_center))
        first_success = None
        for step in range(1, args.max_steps + 1):
            action = actor(state.obs['state'], jax.random.PRNGKey(seed * 1000 + step))[0]
            state = step_fn(state, action)
            if bool(state.metrics['forward/success']) and first_success is None:
                first_success = step
            if bool(state.done):
                break
        R = matrix(np.asarray(state.data.qpos[-4:]))
        top, front = face_toward(R, initial_up), face_toward(R, initial_front)
        row = dict(seed=seed, offset_mm=offset_mm.tolist(), steps=step,
                   first_direction_success_step=first_success,
                   top_id=top, top_color=COLORS[top],
                   front_id=front, front_color=COLORS[front],
                   desired_red_top_green_front=(top == 2 and front == 5),
                   signed_forward_tip_deg=float(state.metrics['forward/tip_deg']),
                   side_yaw_deg=float(state.metrics['forward/yaw_deg']),
                   center_shift_mm=float(1000 * state.metrics['forward/center_shift_m']),
                   done=bool(state.done))
        rows.append(row)
        print(json.dumps(row), flush=True)
    report = dict(status='offline_directional_sac_pilot_eval', motor_commands_sent=0,
                  checkpoint=str(checkpoint_paths[-1]), tested_seeds=len(rows),
                  directional_successes=sum(r['first_direction_success_step'] is not None for r in rows),
                  color_pair_successes=sum(r['desired_red_top_green_front'] for r in rows),
                  results=rows,
                  caveat='Deterministic rollouts in a same-run-fitted but uncalibrated contact model. Original camera occlusion and external wood support are not represented. Not hardware validated.')
    (ROOT / args.run / 'directional_eval_result.json').write_text(
        json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'results'}, indent=2))


if __name__ == '__main__':
    main()
