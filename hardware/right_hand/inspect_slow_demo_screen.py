import json
from pathlib import Path
from slow_grasp_demo_plan import SlowGraspDemoPlan

root=Path(__file__).resolve().parent/'right_hand_rotation_20261023/slow_grasp_demo_fixture'
data=json.loads((root/'native_screen.json').read_bytes())
print('ACCEPTED',data['accepted'])
failed=[]
for row in data['static_screens']:
    bad={}
    for field in ('self_escape_max_unapproved_overlap_mm','max_support_increase_mm',
                  'thumb_support_overlap_mm','unapproved_native_range_violation_rad'):
        if row.get(field,0)>1e-6:bad[field]=row[field]
    for field in ('native_reentry_monotonic','native_reentry_complete','self_escape_complete'):
        if row.get(field) is False:bad[field]=False
    travel=row.get('contact_clear_pip_travel_ticks')
    if travel is None or travel>SlowGraspDemoPlan.MAX_CONTACT_RELEASE_TRAVEL_TICKS:bad['clear_travel']=travel
    if bad:failed.append(dict(offset=row['offset_ticks'],bad=bad,
        complete_at=row.get('self_escape_complete_at'),maximum_step_increase=row.get('self_escape_max_step_increase_mm'),
        initial=row.get('initial_self_contacts_mm')))
print(json.dumps(failed[:10],indent=2));print('FAILED COUNT',len(failed))
