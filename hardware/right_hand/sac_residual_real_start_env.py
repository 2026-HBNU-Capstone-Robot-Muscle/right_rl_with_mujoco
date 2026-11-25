"""JAX residual correction around the existing exported policy, offline only."""

from pathlib import Path

import jax.numpy as jp
import numpy as np

ROOT = Path(__file__).resolve().parent
POLICY = ROOT.parent / 'work' / 'Leap_Test' / 'policy' / 'policy_np.npz'


class JaxTeacher:
    def __init__(self, path=POLICY):
        with np.load(path) as archive:
            self.mean = jp.asarray(archive['mean'])
            self.std = jp.asarray(archive['std'])
            self.action_size = int(archive['action_size'])
            layers = {}
            for key in archive.files:
                if not key.startswith('W:'):
                    continue
                names = [part for part in key[2:].split('/') if part.startswith('hidden_')]
                if names:
                    number = int(names[0].split('_')[1])
                    layers.setdefault(number, {})[key.rsplit('/', 1)[-1]] = jp.asarray(archive[key])
            self.layers = tuple((layers[i]['kernel'], layers[i]['bias'])
                                for i in sorted(layers))

    def __call__(self, obs):
        x = (obs - self.mean) / self.std
        for index, (weight, bias) in enumerate(self.layers):
            x = x @ weight + bias
            if index < len(self.layers) - 1:
                x = jp.maximum(x, 0.)
        return jp.tanh(x[:self.action_size])


class ResidualFaceUp:
    def __init__(self, env, scale=.3):
        self.env = env
        self.teacher = JaxTeacher()
        self.scale = scale

    def __getattr__(self, name):
        return getattr(self.env, name)

    def reset(self, rng):
        return self.env.reset(rng)

    def step(self, state, residual_action):
        teacher_action = self.teacher(state.obs['state'])
        action = jp.clip(teacher_action + self.scale * residual_action, -1., 1.)
        return self.env.step(state, action)
