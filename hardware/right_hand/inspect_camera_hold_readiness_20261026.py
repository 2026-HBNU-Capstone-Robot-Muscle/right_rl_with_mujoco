"""Summarize existing offline proof and device-free reviews."""
import json
from thumb_clear_root_empty_plan_20261026 import ROOT,FIXTURE,ThumbRootEmptyPlan
def main():
    p=ThumbRootEmptyPlan(FIXTURE/'request.json')
    print('FIXTURE',FIXTURE.name)
    for name in ('off_process_review.json','readiness_review.json'):
        path=FIXTURE/name
        print(name,json.loads(path.read_bytes()) if path.exists() else 'absent')
    r=json.loads((FIXTURE/'screen.json').read_bytes())
    print(json.dumps(dict(status=r.get('status'),accepted=r.get('accepted'),static=len(r.get('static_screens',[])),normal=len(r.get('dynamic_sensitivity',[])),camera_holds=len(r.get('camera_gap_hold_sensitivity',[])),hold_policy=r.get('camera_gap_hold_policy'),support_extra=r.get('support_escape_dynamic_extra_seconds'),goals=len(p.plan(p.source_position())),keys=list(r)),ensure_ascii=False))
    for name in ('dynamic_sensitivity','camera_gap_hold_sensitivity'):
        print(name,json.dumps([dict(front=v.get('front'),gain=v.get('gain_scale'),passed=v.get('passed'),camera_hold_events=v.get('camera_hold_events'),support_escape=v.get('support_escape_seconds')) for v in r.get(name,[])],ensure_ascii=False))
    print('SOURCE',json.dumps(json.loads((FIXTURE/'request.json').read_bytes()),ensure_ascii=False)[:2600])
if __name__=='__main__':main()
