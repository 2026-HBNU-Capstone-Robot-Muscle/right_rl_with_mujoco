"""Run the existing SAC trainer with the offline real-start environment.

This entrypoint monkeypatches only its Python process. It does not modify the
installed playground, the read-only work tree, or any live hand controller.
"""

import sys
from pathlib import Path

from mujoco_playground import registry

from sac_real_start_env import RealStartFaceUp

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'train' / 'learning'))

original_load = registry.load


def load_real_start(name, *args, **kwargs):
    env = original_load(name, *args, **kwargs)
    return RealStartFaceUp(env) if name == 'LeapCubeFaceUp' else env


registry.load = load_real_start

from train_jax_sac import run


if __name__ == '__main__':
    run()
