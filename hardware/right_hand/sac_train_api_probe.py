"""Read-only installed Brax SAC API/config audit."""

import inspect

from brax.training.agents.sac import train as sac
from mujoco_playground.config import manipulation_params

print('signature:', inspect.signature(sac.train))
cfg = manipulation_params.brax_sac_config('LeapCubeFaceUp', 'jax')
print('config:', {key: str(value) for key, value in cfg.items()})
