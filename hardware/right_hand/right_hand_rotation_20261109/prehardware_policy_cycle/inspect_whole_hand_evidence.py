"""Compact saved-result inspection; never executes a model or opens a device."""
from pathlib import Path
import json
import sys

BASE=Path(__file__).resolve().parent/'next_cycle/handoff_refinement_20261110'
SKIP={'trace','terminal_guard','by_geom','contact_hand_geoms','loaded_hand_geoms','chosen_velocity','planning_pair_gaps_mm','joint_targets','rows','model_provenance','source_model_provenance'}


def trim(value,depth=0):
    if isinstance(value,dict):
        return {k:trim(v,depth+1) for k,v in value.items() if k not in SKIP and (depth<2 or not isinstance(v,(list,dict)))}
    if isinstance(value,list) and len(value)>8:
        return {'items':len(value)}
    if isinstance(value,list):
        return [trim(v,depth+1) for v in value]
    return value


if __name__=='__main__':
    for name in sys.argv[1:]:
        p=BASE/name/'review.json'
        if not p.exists():
            p=BASE/name/'progress.json'
        if not p.exists():
            print(name, 'missing')
            continue
        report=json.loads(p.read_text(encoding='utf-8'))
        result=trim(report)
        if 'rows' in report:
            result['rows']=[trim(r) for r in report['rows']]
        print(json.dumps({name:result},ensure_ascii=False,indent=2))
