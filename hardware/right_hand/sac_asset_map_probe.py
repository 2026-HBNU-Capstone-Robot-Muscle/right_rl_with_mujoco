"""Inspect the installed simulator's XML asset keys without device access."""
import json
import jax
from mujoco_playground import registry

cfg = registry.get_default_config('LeapCubeFaceUp')
cfg.impl = 'jax'
env = registry.load('LeapCubeFaceUp', config=cfg)
_ = env.reset(jax.random.PRNGKey(0))
print(json.dumps({'xml_path': str(env.xml_path),
                  'xml_asset_keys': [str(k) for k in env._model_assets if str(k).endswith('.xml')],
                  'mesh_asset_count': sum(str(k).endswith(('.obj', '.stl')) for k in env._model_assets)}, indent=2))
