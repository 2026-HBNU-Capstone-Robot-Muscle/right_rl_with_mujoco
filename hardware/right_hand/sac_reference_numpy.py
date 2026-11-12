"""Offline deterministic SAC/reference inference. No camera or motor access."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
DEFAULT = ROOT / 'sac_forward_support_reference_v1_20k' / 'numpy_export'


def vector(value, length, name):
    value = np.asarray(value, dtype=np.float32)
    if value.shape != (length,) or not np.isfinite(value).all():
        raise ValueError(f'{name}: expected {length} finite values')
    return value


class ReferencePolicy:
    def __init__(self, folder=DEFAULT):
        folder = Path(folder)
        self.manifest = json.loads((folder / 'manifest.json').read_text())
        if self.manifest['format'] != 'leap_reference_sac_numpy_v1':
            raise ValueError('Unsupported export format')
        import hashlib
        if hashlib.sha256((folder / 'actor.npz').read_bytes()).hexdigest() != self.manifest['actor_sha256']:
            raise ValueError('Actor archive hash mismatch')
        with np.load(folder / 'actor.npz', allow_pickle=False) as z:
            self.mean = z['mean']; self.std = z['std']
            self.layers = [(z[f'w{i}'], z[f'b{i}']) for i in range(3)]
            self.base = z['base']; self.targets = z['targets']
            self.lower = z['lower']; self.upper = z['upper']
        if (self.mean.shape != (110,) or self.std.shape != (110,)
                or not np.isfinite(self.std).all() or np.any(self.std <= 0)):
            raise ValueError('Invalid normalization statistics')

    def action(self, observation):
        x = (vector(observation, 110, 'observation') - self.mean) / self.std
        for i, (w, b) in enumerate(self.layers):
            x = x @ w + b
            if i < len(self.layers) - 1:
                x = np.maximum(x, np.float32(0))
        return vector(np.tanh(x[:16]), 16, 'policy output')

    def reference(self, step):
        if not isinstance(step, (int, np.integer)) or step < 0:
            raise ValueError('step must be a nonnegative integer')
        elapsed = np.float32(step + 1) * np.float32(.05)
        phase = min(int(np.floor(elapsed / np.float32(1.2))), 2)
        previous = self.base if phase == 0 else self.targets[phase - 1]
        fraction = np.clip((elapsed - np.float32(phase) * np.float32(1.2)) / np.float32(1.2), 0, 1)
        return previous + (self.targets[phase] - previous) * fraction

    def command(self, observation, previous_goal, step):
        """Return simulated joint goals and executed base action; DOES NOT send."""
        previous = vector(previous_goal, 16, 'previous commanded goal')
        action = self.action(observation)
        desired = np.clip(self.reference(step) + np.float32(.05) * action, self.lower, self.upper)
        executed = np.clip((desired - previous) / np.float32(.2), -1, 1)
        goal = np.clip(previous + np.float32(.2) * executed, self.lower, self.upper)
        return vector(goal, 16, 'goal'), executed


def check(folder=DEFAULT):
    folder = Path(folder)
    policy = ReferencePolicy(folder)
    import hashlib
    if hashlib.sha256((folder / 'jax_fixtures.npz').read_bytes()).hexdigest() != policy.manifest['fixture_sha256']:
        raise ValueError('Fixture archive hash mismatch')
    with np.load(folder / 'jax_fixtures.npz', allow_pickle=False) as z:
        actual = np.stack([policy.action(o) for o in z['observations']])
        actions_error = float(np.max(np.abs(actual - z['actions'])))
        goals = np.stack([policy.command(o, c, int(s))[0]
                          for o, c, s in zip(z['rollout_observations'], z['previous_goals'], z['steps'])])
        goals_error = float(np.max(np.abs(goals - z['goals'])))
        default_gpu_error = float(np.max(np.abs(actual[len(goals):] - z['random_default_actions'])))
        report = dict(status='passed' if actions_error < 1e-5 and goals_error < 1e-5 else 'failed',
                      action_samples=len(actual), rollout_samples=len(goals),
                      max_action_error=actions_error, max_goal_error_rad=goals_error,
                      random_default_gpu_error=default_gpu_error,
                      motor_commands_sent=0, hardware_validated=False)
    if report['status'] != 'passed':
        raise AssertionError(report)
    return report


if __name__ == '__main__':
    result = check()
    (DEFAULT / 'windows_parity.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
