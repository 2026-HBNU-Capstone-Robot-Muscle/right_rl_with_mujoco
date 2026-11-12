"""Read-only audit of installed Brax SAC checkpoint format."""

import inspect

from brax.training.agents.sac import train as sac
from brax.training.agents.sac import checkpoint

for name in ('load', 'load_policy', 'make_inference_fn'):
    function = getattr(checkpoint, name, None)
    print(name, inspect.signature(function) if function else None)
print('load_policy source:', inspect.getsource(checkpoint.load_policy))

source = inspect.getsource(sac)
lines = source.splitlines()
for index, line in enumerate(lines):
    if 'checkpoint_logdir' in line or 'save_checkpoint' in line or 'restore_checkpoint' in line:
        lo, hi = max(0, index - 3), min(len(lines), index + 8)
        print('\n'.join(f'{number+1}: {lines[number]}' for number in range(lo, hi)))
        print('---')
