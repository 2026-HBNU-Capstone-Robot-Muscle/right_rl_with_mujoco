"""Export the newly trained actor and JAX parity fixtures; offline WSL only."""
import os
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')
import hashlib
import json
from pathlib import Path
import jax
import jax.numpy as jp
import numpy as np
from brax.training.agents.sac import checkpoint
from mujoco_playground import registry
from sac_supported_start_env import SupportedStart, SupportedForward
from sac_reference_residual_env import ReferenceResidual

ROOT = Path(__file__).resolve().parent
RUN = ROOT / 'sac_forward_support_reference_v1_20k'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    settings = json.loads((RUN / 'settings.json').read_text())
    ckpt = RUN / 'checkpoints' / '000000020000'
    config = json.loads((ckpt / 'sac_network_config.json').read_text())
    kw = config['network_factory_kwargs']
    if (config['observation_size'] != 110 or config['action_size'] != 16
            or not config['normalize_observations'] or kw['activation'] != 'relu'
            or kw['hidden_layer_sizes'] != [256, 256] or kw['policy_network_layer_norm']
            or kw['distribution_type'] != 'tanh_normal'):
        raise RuntimeError('Unsupported checkpoint architecture')
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'; cfg.chain_goals = False; cfg.face_onehot_obs = False
    env = ReferenceResidual(SupportedForward(SupportedStart(registry.load('LeapCubeFaceUp', config=cfg))),
                            settings['reference_residual'], settings['reference_scale'])
    for key in ('reference_sha256', 'reset_manifest_sha256', 'dynamics_version'):
        if settings[key] != getattr(env, key):
            raise RuntimeError('Environment mismatch: ' + key)
    if env.scale != .05 or env.dt != .05 or env._config.action_scale != .2:
        raise RuntimeError('Unsupported command timing/scale')
    norm, weights = checkpoint.load(str(ckpt))
    params = dict(mean=np.asarray(norm.mean), std=np.asarray(norm.std), base=np.asarray(env.base),
                  targets=np.asarray(env.targets), lower=np.asarray(env._safe_lowers, dtype=np.float32),
                  upper=np.asarray(env._safe_uppers, dtype=np.float32))
    for i in range(3):
        layer = weights['params'][f'hidden_{i}']
        params[f'w{i}'] = np.asarray(layer['kernel']); params[f'b{i}'] = np.asarray(layer['bias'])
    out = RUN / 'numpy_export'; out.mkdir(exist_ok=True)
    np.savez(out / 'actor.npz', **params)
    actor = checkpoint.load_policy(str(ckpt), deterministic=True)
    print('Exported actor. Generating observations from an actual 90-step MJX rollout.', flush=True)
    state = jax.jit(env.reset_at)(jax.random.PRNGKey(0), jp.asarray(1))

    @jax.jit
    def rollout(state):
        def body(state, step):
            obs = state.obs['state']
            action = actor(obs, jax.random.PRNGKey(0))[0]
            new = env.step(state, action)
            row = (obs, action, state.data.ctrl, new.data.ctrl, step)
            return new, row
        return jax.lax.scan(body, state, jp.arange(90))

    _, rows = rollout(state)
    obs, action, previous, goals, steps = map(np.asarray, rows)
    # In addition to real rollout inputs, exercise broader normalized inputs.
    rng = np.random.default_rng(20261013)
    random_obs = (params['mean'] + params['std'] * rng.normal(size=(512, 110))).astype(np.float32)
    random_default = np.asarray(jax.jit(jax.vmap(lambda o: actor(o, jax.random.PRNGKey(0))[0]))(random_obs))
    with jax.default_matmul_precision('highest'):
        random_actions = np.asarray(jax.jit(jax.vmap(lambda o: actor(o, jax.random.PRNGKey(0))[0]))(random_obs))
    np.savez(out / 'jax_fixtures.npz', observations=np.concatenate([obs, random_obs]),
             actions=np.concatenate([action, random_actions]), random_default_actions=random_default, rollout_observations=obs,
             previous_goals=previous, goals=goals, steps=steps)
    files = sorted(p for p in ckpt.rglob('*') if p.is_file())
    manifest = dict(format='leap_reference_sac_numpy_v1', checkpoint='000000020000',
                    checkpoint_files_sha256={str(p.relative_to(ckpt)): digest(p) for p in files},
                    actor_sha256=digest(out / 'actor.npz'), fixture_sha256=digest(out / 'jax_fixtures.npz'),
                    observation_size=110, action_size=16, control_dt=.05, reference_waypoint_seconds=1.2,
                    residual_rad=.05, base_action_scale_rad=.2,
                    reset_manifest_sha256=env.reset_manifest_sha256, reference_sha256=env.reference_sha256,
                    dynamics_version=env.dynamics_version, hardware_validated=False,
                    fixture_note='90 consecutive MJX steps use original GPU inference; 512 random inputs use highest matmul precision. Default GPU random outputs retained separately. Parity, not a new success evaluation.')
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    from sac_reference_numpy import check
    result = check(out)
    (out / 'wsl_numpy_parity.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
