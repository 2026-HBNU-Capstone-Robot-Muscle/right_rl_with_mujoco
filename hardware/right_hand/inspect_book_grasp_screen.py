"""Read only the offline native screen's failed criteria."""
import json
from collections import Counter
from book_grasp_trial_plan import ROOT
from book_grasp_trial_gate import screen_failure_details
import sys
folder=ROOT/'right_hand_rotation_20261024/book20_current_review'/('grasp_route_fixture_v2' if len(sys.argv)==1 else sys.argv[1])
if len(sys.argv)>1 and (ROOT/sys.argv[1]).is_dir():folder=ROOT/sys.argv[1]
p=folder/(sys.argv[2] if len(sys.argv)>2 else 'screen.json');r=json.loads(p.read_bytes())
print(screen_failure_details(p))
from math import isfinite
from pprint import pprint
for name in ('static_screens','dynamic_sensitivity'):
    rows=r[name];bad=[]
    for s in rows:
        if (s.get('max_self_after_release_mm',s['max_self_overlap_mm'])>1e-6 or s['max_support_after_lift_mm']>.15
            or s['support_escape_max_unapproved_overlap_mm']>1e-6 or s['native_range_violation_rad']>1e-6
            or not s['support_escape_complete'] or s.get('max_tracking_rad',0)>.14
            or s.get('self_escape_max_unapproved_overlap_mm',0)>1e-6 or s.get('self_escape_complete') is False):bad.append(s)
    print(name,len(bad),'/',len(rows))
    if bad:
        for front in (-15,-10,-5):
            chosen=next((s for s in bad if s['book_front_mm']==front),None)
            if chosen: print(json.dumps(chosen))
