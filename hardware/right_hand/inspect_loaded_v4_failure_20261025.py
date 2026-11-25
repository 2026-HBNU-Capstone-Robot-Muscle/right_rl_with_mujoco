import json
from pathlib import Path
r=json.loads((Path(__file__).resolve().parent/'right_hand_rotation_20261024/loaded_quarter_trial_20261025/fixture_v4/screen.json').read_bytes())
for k in ('static_screens','dynamic_sensitivity'):
    print(k)
    for field in ('max_self_after_release_mm','self_escape_max_unapproved_overlap_mm','max_support_after_lift_mm','support_escape_max_unapproved_overlap_mm','native_cmc_after_release_min_ticks','native_range_violation_rad'):
        rows=sorted(r[k],key=lambda s:s[field]);s=rows[0] if field.startswith('native_cmc') else rows[-1]
        print(field,s[field],s.get('book_front_mm'),s.get('offset_ticks'),s.get('gain_scale'))
    if k=='dynamic_sensitivity':
        for s in r[k]:print('CMC',s['book_front_mm'],s['gain_scale'],s['native_cmc_after_release_min_ticks'],s['initial_self_contacts_mm'])
