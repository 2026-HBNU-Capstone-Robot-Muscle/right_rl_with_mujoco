import json
from recovered_forward_plan_20261026 import FIXTURE
def main():
    r=json.loads((FIXTURE/'screen.json').read_bytes())
    keys=('book_front_mm','gain_scale','offset_ticks','self_escape_complete','support_escape_complete','self_escape_complete_at','self_escape_deadline','self_escape_max_unapproved_overlap_mm','self_escape_max_step_increase_mm','max_self_after_release_mm','max_ring_self_after_release_mm','max_support_after_lift_mm','support_escape_max_unapproved_overlap_mm','support_escape_complete_at','support_escape_deadline','native_range_violation_rad','unmodified_joint_range_violation_rad','native_cmc_after_release_min_ticks','max_tracking_rad','max_tracking_after_clearance_rad','initial_self_contacts_mm')
    for group in ('static_screens','dynamic_sensitivity'):
        cases=r[group]
        if group=='static_screens':cases=[s for s in cases if s['offset_ticks']==[0]*16]
        print(group)
        for s in cases:print(json.dumps({k:s.get(k) for k in keys if k in s},ensure_ascii=False))
if __name__=='__main__':main()
