import json
from pathlib import Path
root=Path(__file__).resolve().parent/'right_hand_rotation_20261024/book20_current_review/preparation_fixture'
for path in sorted(root.glob('screen*.json')):
    r=json.loads(path.read_bytes())
    print(path.name,'accepted',r['accepted_for_supervised_empty_preparation'],
          'seconds_after_request_saved',round(path.stat().st_mtime-(root/'request.json').stat().st_mtime,1))
    print('initial pairs',r['static_screens'][0]['initial_support_contacts_mm'])
    print('unique pairs',sorted({k for s in r['static_screens'] for k in s['initial_support_contacts_mm']}))
    for collection in ('static_screens','dynamic_sensitivity'):
        rows=r[collection]
        print(collection,{key:max(s[key] for s in rows) for key in
            ('max_self_overlap_mm','max_support_after_lift_mm','support_escape_max_unapproved_overlap_mm',
             'support_escape_max_step_increase_mm','native_range_violation_rad')})
        print('complete at max',max((s['support_escape_complete_at'] or 0) for s in rows))
        if collection=='dynamic_sensitivity':
            print('dynamic first',rows[0])
