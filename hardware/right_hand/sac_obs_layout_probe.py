"""Offline introspection of installed simulated SAC observation layout."""

import inspect

from mujoco_playground import registry


config = registry.get_default_config('LeapCubeFaceUp')
config.impl = 'jax'
env = registry.load('LeapCubeFaceUp', config=config)
print('observation_size:', env.observation_size)
for parent in type(env).__mro__:
    if '_get_obs' in parent.__dict__:
        print('class:', parent.__name__)
        print(inspect.getsource(parent._get_obs))
