"""Summarize only rejected native cases for the repeat reference."""
import json
from loaded_repeat_plan_20261026 import FIXTURE
def main():
    r=json.loads((FIXTURE/'screen.json').read_bytes())
    for front in (-15.,-10.,-5.):
        group=[s for s in r['static_screens'] if s['book_front_mm']==front]
        print('STATIC_CLEAR',front,'max_goal',max(s['support_escape_complete_at'] for s in group),'nominal_goal',next(s['support_escape_complete_at'] for s in group if s['offset_ticks']==[0]*16))
    for name in ('static_screens','dynamic_sensitivity','camera_gap_hold_sensitivity'):
        bad=[v for v in r[name] if not v['support_escape_complete'] or v['support_escape_max_unapproved_overlap_mm']>1e-6 or not v['self_escape_complete'] or v['self_escape_max_unapproved_overlap_mm']>1e-6]
        print(name,len(bad))
        for row in bad[:4]:print(json.dumps(row))
if __name__=='__main__':main()
