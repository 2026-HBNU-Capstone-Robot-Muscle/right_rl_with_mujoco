"""Offline inspection of installed SAC inference contract."""
import inspect
import json
from pathlib import Path
import jax
from brax.training.agents.sac import checkpoint, networks
from brax.training.acme import running_statistics
from mujoco_playground import registry

root = Path(__file__).resolve().parent
folder = root / 'sac_forward_support_reference_v1_20k'
ckpt = folder / 'checkpoints' / '000000020000'
parts = [inspect.getsource(checkpoint), inspect.getsource(networks.make_inference_fn),
         inspect.getsource(running_statistics.normalize)]
cfg = registry.get_default_config('LeapCubeFaceUp')
cfg.impl = 'jax'
env = registry.load('LeapCubeFaceUp', config=cfg)
parts += [inspect.getsource(cls._get_obs) for cls in type(env).__mro__ if '_get_obs' in cls.__dict__]
parts += [inspect.getsource(cls.step) for cls in type(env).__mro__ if 'step' in cls.__dict__]
(folder / 'installed_inference_contract.txt').write_text('\n\n'.join(parts))
params = checkpoint.load(str(ckpt))
print(jax.tree_util.tree_map(lambda x: {'shape': list(x.shape), 'dtype': str(x.dtype)}, params))
print('Checkpoint files:', [p.name for p in ckpt.iterdir()])

