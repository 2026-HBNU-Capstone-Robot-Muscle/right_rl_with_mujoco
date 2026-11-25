import json
from pathlib import Path
p=Path(__file__).resolve().parent/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/fixture_v1/screen.json'
r=json.loads(p.read_bytes())
for k in ('static_screens','dynamic_sensitivity'):
    print(k)
    for field in ('max_self_after_release_mm','self_escape_max_unapproved_overlap_mm','max_support_after_lift_mm','max_ring_self_after_release_mm'):
        a=max(r[k],key=lambda s:s[field]);print(field,a[field],a.get('offset_ticks'),a.get('gain_scale'))
print('INITIAL',r['static_screens'][0]['initial_self_contacts_mm'])
