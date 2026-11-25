"""One offline JAX step and exact-rotation sign checks for the forward task."""

import json
import argparse

import jax
import jax.numpy as jp
from mujoco_playground import registry

from sac_forward_task_env import ForwardFirstFlip, direction_components
from sac_real_start_env import RealStartFaceUp
from sac_stage2_start_env import Stage2StartFaceUp
from sac_stage0_start_env import Stage0StartFaceUp
from sac_forward_geometry import measure, target_quaternion
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage2', action='store_true')
    args = parser.parse_args()
    half = jp.sqrt(.5)
    identity = jp.asarray([1., 0., 0., 0.])
    positive = jp.asarray([half, 0., 0., -half])
    negative = jp.asarray([half, 0., 0., half])
    positive_tip = float(jp.rad2deg(direction_components(identity, positive)[0]))
    negative_tip = float(jp.rad2deg(direction_components(identity, negative)[0]))
    assert abs(positive_tip - 90) < 1e-4
    assert abs(negative_tip + 90) < 1e-4
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    cfg.chain_goals=False
    cfg.face_onehot_obs=False
    base = registry.load('LeapCubeFaceUp', config=cfg)
    start_env = Stage2StartFaceUp(base) if args.stage2 else Stage0StartFaceUp(base)
    env = ForwardFirstFlip(start_env)
    state = env.reset(jax.random.PRNGKey(0))
    np.testing.assert_allclose(np.asarray(state.data.mocap_quat[0]),
                               target_quaternion(np.asarray(state.data.qpos[-4:])),atol=1e-6)
    # Validate the exact JAX geometry used by the environment, including float32.
    geometry=jax.jit(lambda q: measure(identity,q,xp=jp))
    for deg in (89.,90.,91.,100.):
        a=jp.deg2rad(deg)/2
        values=geometry(jp.asarray([jp.cos(a),0.,0.,-jp.sin(a)]))
        np.testing.assert_allclose(np.rad2deg(np.asarray(values)),[deg,0,abs(deg-90)],atol=1e-4)
    stepped = jax.jit(env.step)(state, jp.zeros(env.action_size))
    report = dict(status='offline_forward_task_smoke', motor_commands_sent=0,
                  analytic_positive_tip_deg=positive_tip,
                  analytic_negative_tip_deg=negative_tip,
                  stage2_same_run_start=args.stage2,
                  reward=float(stepped.reward), done=float(stepped.done),
                  forward_tip_deg=float(stepped.metrics['forward/tip_deg']),
                  forward_off_axis_deg=float(stepped.metrics['forward/off_axis_deg']),
                  target_error_deg=float(stepped.metrics['forward/target_error_deg']),
                  forward_success=float(stepped.metrics['forward/success']),
                  caveat='One JAX step and analytic sign checks only. Start/contact model and full SAC training unvalidated.')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
