import json
from pathlib import Path
from reverse_start_empty_plan_20261025 import ReverseStartPlan,ROOT
from reverse_start_empty_gate_20261025 import validate_proof
from camera_stream import enumerate_names

folder=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/fixture_v2'
p=ReverseStartPlan(folder/'request.json');r=validate_proof(p,folder/'screen.json')
print('EMPTY_READINESS',len(r['static_screens']),len(r['dynamic_sensitivity']),len(p.plan(p.source_position())))
try:print('CAMERA_INVENTORY',json.dumps(enumerate_names(),ensure_ascii=False))
except Exception as exc:print('CAMERA_INVENTORY_UNAVAILABLE',str(exc))
