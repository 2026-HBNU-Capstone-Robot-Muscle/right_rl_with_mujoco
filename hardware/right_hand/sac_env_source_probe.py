"""Read-only pointer to installed task implementation."""

import inspect

from mujoco_playground import registry

config = registry.get_default_config('LeapCubeFaceUp')
config.impl = 'jax'
env = registry.load('LeapCubeFaceUp', config=config)
print(inspect.getfile(type(env)))
for name in ('step', '_get_reward', '_get_termination', '_cube_orientation_error'):
    method = getattr(type(env), name, None)
    if method is not None:
        print(name, inspect.getfile(method), inspect.getsourcelines(method)[1])
