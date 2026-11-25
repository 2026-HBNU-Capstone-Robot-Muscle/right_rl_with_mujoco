"""Offline export of actual native root-joint descendants, not a pose fit."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
from mujoco_playground import registry
from book_support_model_20261024 import build_book
from book_support_contact_scope import derive_scope,SCOPE_PATH

root=Path(__file__).resolve().parent
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
model,_=build_book(env,root/'right_hand_rotation_20261024/book20_current_review/scope_scene.xml',
    book_projection_mm=-10.,book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
scope=derive_scope(model)
if SCOPE_PATH.exists() and json.loads(SCOPE_PATH.read_bytes())!=scope:
    raise ValueError('Native hierarchy changed; refuse to overwrite an earlier scope')
SCOPE_PATH.write_text(json.dumps(scope,indent=2),encoding='utf-8')
print(json.dumps(dict(scope_file=str(SCOPE_PATH),groups={k:dict(root_joint=v['root_joint'],root_body=v['root_body'],
    geometry_count=len(v['geometry_names'])) for k,v in scope['root_groups'].items()},motor_commands_sent=0),indent=2),flush=True)
