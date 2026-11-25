"""Export unchanged model ranges for bounded OFF re-entry admission only."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build_with_middle_thumb_hulls(env)
lo=np.zeros(16);hi=np.zeros(16)
for i,motor in enumerate(SIM_TO_MOTOR_IDS):
    joint=m.actuator_trnid[i,0]
    assert m.jnt_limited[joint] and m.actuator_ctrllimited[i]
    lo[motor-1]=(max(m.jnt_range[joint,0],m.actuator_ctrlrange[i,0])+np.pi)/TICK
    hi[motor-1]=(min(m.jnt_range[joint,1],m.actuator_ctrlrange[i,1])+np.pi)/TICK
p=Path(__file__).resolve().parent/'right_hand_rotation_20261023/native_motor_limits.json'
p.write_text(json.dumps(dict(lower_ticks=lo.tolist(),upper_ticks=hi.tolist(),tick_rad=TICK,
    native_ranges_unchanged=True,hardware_commands_sent=0),indent=2),encoding='utf-8')
print(p.name)
