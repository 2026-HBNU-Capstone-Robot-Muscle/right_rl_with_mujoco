"""Compact current offline findings."""
import json,sys
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8');R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip'
for name in ('continuous_handoff/review.json','supported_handoff_first_full/review.json','handoff_preload/normal0.8/review.json'):
    p=O/name;d=json.loads(p.read_bytes());print(name)
    for k,v in d.items():
        if k in ('dynamic','segments','checkpoints','cases','source_bindings','loads','checkpoint_reproduction'):continue
        if not isinstance(v,(list,dict)):print(k,v)
        elif k not in ('trace',):print(k,str(v)[:1500])
