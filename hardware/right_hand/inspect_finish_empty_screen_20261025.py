import json
from pathlib import Path
from quarter_finish_empty_plan_20261025 import FIXTURE
p=FIXTURE/'screen.json';r=json.loads(p.read_bytes())
keys=['max_self_after_release_mm','max_support_after_lift_mm','native_range_violation_rad','unmodified_joint_range_violation_rad','support_escape_max_unapproved_overlap_mm','self_escape_max_unapproved_overlap_mm','support_escape_complete','self_escape_complete','max_ring_self_after_release_mm','native_cmc_after_release_min_ticks','max_tracking_after_clearance_rad','support_escape_max_step_increase_mm','self_escape_max_step_increase_mm']
for group in ('static_screens','dynamic_sensitivity'):
    a=r[group];print(group,len(a))
    print('summary',{k:max(float(x[k]) for x in a if isinstance(x.get(k),(float,int))) for k in keys if isinstance(a[0].get(k),(float,int))})
    for x in a[:1]+a[-1:]:print(json.dumps({k:v for k,v in x.items() if any(t in k for t in ('contact','escape','after','cmc','offset','front','gain'))},ensure_ascii=False))
