"""Inspect installed SAC APIs and wrapper contracts without training."""
from pathlib import Path
import inspect
from brax.training.agents.sac import train, checkpoint
from mujoco_playground import wrapper
from mujoco_playground._src import mjx_env

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261109/prehardware_policy_cycle'
OUT.mkdir(parents=True, exist_ok=True)
items = {'train_signature': str(inspect.signature(train.train)),
         'state': inspect.getsource(mjx_env.State),
         'training_wrapper': inspect.getsource(wrapper.wrap_for_brax_training),
         'checkpoint_load': inspect.getsource(checkpoint.load_policy)}
for key, value in items.items():
    (OUT / (key + '.txt')).write_text(value, encoding='utf-8')
(OUT / 'installed_sac_train.py').write_text(inspect.getsource(train), encoding='utf-8')
print(items['train_signature'], flush=True)
