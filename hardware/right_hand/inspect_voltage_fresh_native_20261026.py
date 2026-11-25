"""Summarize the new OFF reference's native geometry failures."""
import json
from thumb_clear_root_empty_plan_20261026 import FIXTURE
def main():
    r=json.loads((FIXTURE/'screen.json').read_bytes())
    for kind in ('static_screens','dynamic_sensitivity','camera_gap_hold_sensitivity'):
        values=r.get(kind,[]);bad=[]
        for s in values:
            if (not s.get('self_escape_complete') or s.get('self_escape_max_unapproved_overlap_mm',0)>1e-6
                or not s.get('support_escape_complete') or s.get('support_escape_max_unapproved_overlap_mm',0)>1e-6
                or s.get('max_self_after_release_mm',0)>1e-6 or s.get('max_support_after_lift_mm',0)>.15):bad.append(s)
        print(kind,'count',len(values),'bad',len(bad))
        for s in bad[:2]:print(json.dumps({k:s.get(k) for k in ('book_front_mm','gain_scale','offset_ticks','initial_self_contacts_mm','self_escape_max_unapproved_overlap_mm','self_escape_max_step_increase_mm','self_escape_complete_at','self_escape_complete')},ensure_ascii=False))
    print('overall',r.get('accepted_for_supervised_empty_preparation'))
    from thumb_clear_root_empty_plan_20261026 import ROOT
    folder=ROOT/'thumb_clear_root_empty_20261026_115050_905503/camera'
    frames=json.loads((folder/'frames.json').read_bytes())['frames']
    print('LAST_IMAGES',[str(folder/f['image']) for f in frames[-3:]])
if __name__=='__main__':main()
