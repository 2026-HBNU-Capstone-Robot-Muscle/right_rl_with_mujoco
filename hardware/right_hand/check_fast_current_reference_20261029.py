"""Compare saved fresh OFF reads to the screened start without opening devices."""
import hashlib,json
import numpy as np
from retained_slow_yaw_plan_20261029 import ROOT,OUT,REQUEST,RetainedSlowYawPlan
source=ROOT/'fast_off_readonly_20261029_025957_980112/result.json';r=json.loads(source.read_bytes())
assert r['status']=='off_snapshots_saved' and r['servo_writes']==0 and r['torque_changes']==0 and r['maximum_drift_ticks']<=2
p=RetainedSlowYawPlan(REQUEST);values=np.asarray([s['position'] for s in r['snapshots']]);difference=values-p.source_position()
review=dict(status='fresh_OFF_matches_screened_start' if np.max(abs(difference))<=4 else 'fresh_review_needed',
    source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    reference_start=p.source_position().tolist(),latest_start=values[-1].tolist(),difference_ticks=difference.tolist(),
    maximum_difference_ticks=int(np.max(abs(difference))),existing_radius_ticks=4,cube_removed_confirmed_by_user=True,assistant_motor_writes=0)
(OUT/'latest_fast_OFF_comparison.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
print(json.dumps(review))
if review['status']!='fresh_OFF_matches_screened_start':raise SystemExit(2)
