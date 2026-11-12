"""Read installed task settings and selected source lines; no device access."""
import inspect
import json
from pathlib import Path
from mujoco_playground import registry
cfg=registry.get_default_config('LeapCubeFaceUp'); cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg)
out=Path(__file__).resolve().parent/'sac_task_config_audit.txt'
text='CONFIG\n'+str(cfg)+'\n'
for method in ('_get_obs','reset','_cube_orientation_error'):
    text+='\n'+method+'\n'+inspect.getsource(getattr(type(env),method))
out.write_text(text)
print(str(cfg))
