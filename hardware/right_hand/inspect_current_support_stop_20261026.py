"""Identify the support-release failure after camera-triggered OFF settling."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    p=ROOT/'thumb_clear_root_empty_20261026_111118_102113';r=json.loads((p/'current_start_screen.json').read_bytes())
    print(json.dumps({k:r.get(k) for k in ('start_ticks','plan_version','elapsed_seconds')},indent=2))
    fields=('book_front_mm','gain_scale','support_escape_max_unapproved_overlap_mm','support_escape_max_step_increase_mm','support_escape_complete','support_escape_complete_at','support_escape_deadline','max_support_after_lift_mm','initial_support_contacts_mm')
    for key in ('static_screens','dynamic_sensitivity'):
        bad=[v for v in r[key] if v.get('support_escape_max_unapproved_overlap_mm',0)>1e-6 or not v.get('support_escape_complete',True)]
        print(key,len(bad),'/',len(r[key]))
        print(json.dumps([{k:v.get(k) for k in fields} for v in bad[:3]],indent=2))
if __name__=='__main__':main()
