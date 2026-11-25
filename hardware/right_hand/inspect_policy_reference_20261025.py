import json
from pathlib import Path
names=['sac_reachable_trace_search_result.json','sac_reachable_trace_native_validation.json',
    'sac_reachable_trace_mjx_validation.json','sac_id16_cap3238_native_validation.json',
    'sac_reachable_empty_path.json','right_hand_rotation_20261024/original_sequence_clearance/CAD_tip_contact_comparison_v2.json']
def compact(v,depth=0):
    if isinstance(v,dict):return {k:compact(x,depth+1) for k,x in v.items()}
    if isinstance(v,list) and len(v)>18:return {'count':len(v),'first':compact(v[0],depth+1),'last':compact(v[-1],depth+1)}
    if isinstance(v,list):return [compact(x,depth+1) for x in v]
    return v
for name in names:
    p=Path(name)
    print('\nFILE',name)
    if p.exists():print(json.dumps(compact(json.loads(p.read_bytes())),ensure_ascii=False)[:18000])
    else:print('missing')
