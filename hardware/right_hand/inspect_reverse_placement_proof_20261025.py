import json,sys
from pathlib import Path
r=json.loads(Path(sys.argv[1]).read_bytes())
print('accepted',r['accepted_for_supervised_empty_preparation'],'static',len(r['static_screens']),'native',len(r['dynamic_sensitivity']))
for row in r['dynamic_sensitivity']:
    keys=('max_self_after_release_mm','max_support_after_lift_mm','max_tracking_after_clearance_rad','support_escape_complete','self_escape_complete','support_escape_complete_at','support_escape_deadline')
    if '--brief' not in sys.argv:keys=keys+('initial_support_contacts_mm',)
    print(row['book_front_mm'],row['gain_scale'],{k:v for k,v in row.items() if ('unapproved' in k and v) or k in keys})
