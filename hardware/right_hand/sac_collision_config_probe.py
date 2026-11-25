import inspect,json
from pathlib import Path
import mujoco
from mujoco.mjx._src import collision_driver
from mujoco_playground import registry
from sac_wood_glass_model import build
ROOT=Path(__file__).resolve().parent
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env)
numerics={m.numeric(i).name:m.numeric(i).data.tolist() for i in range(m.nnumeric)}
print(json.dumps(dict(numerics=numerics,npair=m.npair,nexclude=m.nexclude,xml_path=str(env.xml_path)),indent=2))
(ROOT/'sac_collision_driver_source.txt').write_text(inspect.getsource(collision_driver))
(ROOT/'sac_collision_config_probe_result.json').write_text(json.dumps(dict(numerics=numerics,npair=m.npair,nexclude=m.nexclude),indent=2)+'\n')
