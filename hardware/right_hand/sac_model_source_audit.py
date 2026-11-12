"""Inspect local simulator construction to prepare an external support collider."""
import inspect
from pathlib import Path
from mujoco_playground import registry
cfg=registry.get_default_config('LeapCubeFaceUp'); cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg)
text=[]
for cls in type(env).__mro__:
    text.append(str(cls))
    for method in ('__init__','xml_path','_post_init'):
        if method in cls.__dict__:
            fn=cls.__dict__[method]; fn=fn.fget if isinstance(fn,property) else fn
            try:text.append(inspect.getsource(fn))
            except (TypeError,OSError):pass
text.append(str(env.__dict__.keys()))
out=Path(__file__).resolve().parent/'sac_model_source_audit.txt';out.write_text('\n'.join(text))
print(out)
