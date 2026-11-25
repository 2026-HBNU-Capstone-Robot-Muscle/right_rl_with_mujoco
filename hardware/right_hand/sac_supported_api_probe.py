import inspect
from pathlib import Path
from mujoco_playground import registry
ROOT=Path(__file__).resolve().parent
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
text='\n'.join(inspect.getsource(cls.__dict__['step']) for cls in type(env).__mro__ if 'step' in cls.__dict__)
for cls in type(env).__mro__:
    if 'observation_size' in cls.__dict__:text+=str(cls)+'\n'+inspect.getsource(cls.__dict__['observation_size'].fget)
text+='\nCONFIG\n'+str(cfg)
(ROOT/'sac_supported_api_probe.txt').write_text(text)
print(text[:15000])
