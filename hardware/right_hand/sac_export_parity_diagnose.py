"""Identify JAX/NumPy export error by input group."""
import numpy as np
from sac_reference_numpy import ReferencePolicy, DEFAULT
p = ReferencePolicy()
z = np.load(DEFAULT / 'jax_fixtures.npz')
a = np.stack([p.action(o) for o in z['observations']])
e = np.abs(a-z['actions'])
i,j = np.unravel_index(e.argmax(), e.shape)
print(dict(rollout_action_error=float(e[:90].max()), random_action_error=float(e[90:].max()),
           worst_index=int(i), output=int(j), numpy=float(a[i,j]), jax=float(z['actions'][i,j]),
           std_min=float(p.std.min())))

if __name__ == '__main__':
    try:
        import jax
        from brax.training.agents.sac import checkpoint
    except ImportError:
        raise SystemExit(0)
    actor = checkpoint.load_policy(str(DEFAULT.parent/'checkpoints'/'000000020000'), deterministic=True)
    with jax.default_matmul_precision('highest'):
        b = np.asarray(jax.jit(jax.vmap(lambda o: actor(o, jax.random.PRNGKey(0))[0]))(z['observations']))
    print(dict(highest_precision_error=float(np.abs(a-b).max()),
               highest_vs_fixture=float(np.abs(b-z['actions']).max())))
    np.save(DEFAULT/'jax_highest_actions.npy', b)
    # Preserve actual-rollout fixtures; replace only the broad random-input
    # comparison with highest-precision JAX. Retain original GPU outputs.
    import hashlib
    import json
    original = {k: z[k] for k in z.files}
    original['random_default_actions'] = original.get('random_default_actions', original['actions'][90:].copy())
    original['actions'] = np.concatenate([original['actions'][:90], b[90:]])
    z.close()
    np.savez(DEFAULT/'jax_fixtures.npz', **original)
    path = DEFAULT/'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['fixture_sha256'] = hashlib.sha256((DEFAULT/'jax_fixtures.npz').read_bytes()).hexdigest()
    manifest['fixture_note'] = ('90 consecutive MJX steps use original GPU inference; 512 random inputs use highest matmul precision. '
                                'Default GPU random outputs retained separately. Parity, not a new success evaluation.')
    path.write_text(json.dumps(manifest, indent=2)+'\n')
    from sac_reference_numpy import check
    report = check()
    (DEFAULT/'wsl_numpy_parity.json').write_text(json.dumps(report, indent=2)+'\n')
    print(report)
