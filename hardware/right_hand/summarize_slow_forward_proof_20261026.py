import json
from slow_forward_plan_20261026 import FIXTURE
def main():
    r=json.loads((FIXTURE/'screen.json').read_bytes())
    keys=('book_front_mm','gain_scale','self_escape_complete','self_escape_complete_at','self_escape_deadline','self_escape_max_unapproved_overlap_mm','self_escape_max_step_increase_mm','max_self_after_release_mm','max_ring_self_after_release_mm','support_escape_complete','support_escape_complete_at','support_escape_deadline','support_escape_max_unapproved_overlap_mm','max_support_after_lift_mm','native_range_violation_rad','unmodified_joint_range_violation_rad','native_cmc_after_release_min_ticks','max_tracking_rad','max_tracking_after_clearance_rad')
    for s in r['dynamic_sensitivity']:print(json.dumps({k:s.get(k) for k in keys},ensure_ascii=False))
if __name__=='__main__':main()
