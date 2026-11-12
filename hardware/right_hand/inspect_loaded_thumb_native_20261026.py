"""Summarize current loaded-mode hand-route failures, no devices."""
import json
from loaded_thumb_rotation_plan_20261026 import FIXTURE
def main():
    r=json.loads((FIXTURE/'screen.json').read_bytes())
    for kind in ('static_screens','dynamic_sensitivity','camera_gap_hold_sensitivity'):
        bad=[s for s in r.get(kind,[]) if s['self_escape_max_unapproved_overlap_mm']>1e-6 or not s['self_escape_complete'] or not s['support_escape_complete'] or s['support_escape_max_unapproved_overlap_mm']>1e-6]
        print(kind,'bad',len(bad))
        for s in bad[:3]:print(json.dumps({k:s.get(k) for k in ('book_front_mm','offset_ticks','gain_scale','initial_self_contacts_mm','self_escape_max_unapproved_overlap_mm','self_escape_max_step_increase_mm','self_escape_complete_at','self_escape_deadline','self_escape_complete','support_escape_complete')}))
    print('INPUT',r['start_ticks'])
if __name__=='__main__':main()
