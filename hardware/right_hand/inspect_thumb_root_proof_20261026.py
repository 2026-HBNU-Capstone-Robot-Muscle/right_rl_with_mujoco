"""Read failed clearance fields without opening any devices."""
import json
from thumb_clear_root_empty_plan_20261026 import FIXTURE
def main():
    p=FIXTURE/'screen.json'
    if not p.exists():p=FIXTURE/'weak_diagnostic.json'
    r=json.loads(p.read_bytes())
    keys=('book_front_mm','gain_scale','max_self_after_release_mm','max_support_after_lift_mm','native_range_violation_rad','unmodified_joint_range_violation_rad','max_ring_self_after_release_mm','self_escape_max_unapproved_overlap_mm','support_escape_max_unapproved_overlap_mm','max_tracking_rad','max_tracking_after_clearance_rad')
    print(json.dumps(dict(dynamic=[{k:x.get(k) for k in keys} for x in r['dynamic_sensitivity']],static=[r['static_screens'][0]]),indent=2))
if __name__=='__main__':main()
