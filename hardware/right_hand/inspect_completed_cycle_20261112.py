import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
p=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/initial_second_turn_hold_replay/review.json'
r=json.loads(p.read_bytes())
for k,v in r.items():
    if isinstance(v,dict): print(k,json.dumps(v)[:1800])
    elif isinstance(v,list): print(k, 'length',len(v), 'first',json.dumps(v[:1])[:2000])
    else: print(k,repr(v))
