import json
from pathlib import Path
p=Path(__file__).resolve().parent/'right_hand_rotation_20261024/reverse_quarter_resume_empty/fixture_v1/screen.json'
r=json.loads(p.read_bytes())
for kind in ('static_screens','dynamic_sensitivity'):
    print(kind)
    for key in ('max_self_after_release_mm','max_support_after_lift_mm','self_escape_max_unapproved_overlap_mm','support_escape_max_unapproved_overlap_mm','max_ring_self_after_release_mm'):
        row=max(r[kind],key=lambda s:s[key]);print(key,row[key],'front',row['book_front_mm'],'offset',row.get('offset_ticks'))
print('NATIVE_INITIAL_SUPPORT',r['dynamic_sensitivity'][0]['initial_support_contacts_mm'])
print('NATIVE_NOMINAL_SUPPORT',r['dynamic_sensitivity'][0]['nominal_initial_support_contacts_mm'])
print('NATIVE_SELF',r['dynamic_sensitivity'][0]['initial_self_contacts_mm'])
print('PASSIVE_HOLD_CONTACTS',r['dynamic_sensitivity'][0]['passive_enable_hold_support']['contact_maximums_mm'])
