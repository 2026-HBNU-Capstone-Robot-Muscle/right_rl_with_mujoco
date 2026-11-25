import json
from final_grip_pose_plan_20261116 import OUT
for name in ('temporary_support_validation/empty_recovery_screen','temporary_support_validation/full_pose_screen'):
    path=OUT/name/'review.json'
    if not path.exists():continue
    r=json.loads(path.read_bytes())
    print(name,dict(passed=r['all_passed'],statics=sum(x['passed'] for x in r['static_cases']),dynamics=sum(x['passed'] for x in r['dynamic_cases'])))
    print('STATIC_NOMINAL',json.dumps(r['static_cases'][0]))
    print('DYNAMIC_FAILS',[{k:v for k,v in x.items() if k not in ('thumb_index_initial_scope',)} for x in r['dynamic_cases'] if not x['passed']])
