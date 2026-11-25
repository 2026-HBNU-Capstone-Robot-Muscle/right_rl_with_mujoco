"""Small offline diagnostic of the newest saved v2 screen."""
import json
from pathlib import Path
folder=Path(__file__).resolve().parent/'prepared_cube_review_20261022/grasp_trial_v2_fixture'
p=sorted(folder.glob('windows_wsl_screen_*.json'))[-1]
r=json.loads(p.read_bytes())
print(p.name,'accepted',r['accepted'])
print('dynamic',json.dumps(r['dynamic_sensitivity'],indent=2))
for k in ('max_self_overlap_mm','initial_support_overlap_mm','max_support_increase_mm',
          'thumb_support_overlap_mm','max_goal_step_ticks','native_range_violation_rad',
          'unapproved_native_range_violation_rad'):
    print(k,max(s[k] for s in r['static_screens']))
print('reentry',[(s['offset_ticks'],s['native_reentry_monotonic'],s['native_reentry_complete'])
    for s in r['static_screens'] if not s['native_reentry_monotonic'] or not s['native_reentry_complete']])
