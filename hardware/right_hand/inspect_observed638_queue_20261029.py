import json
from adaptive_yaw_loaded_plan_20261029 import AdaptiveYawLoadedPlan,ROOT
F=ROOT/'adaptive_yaw_loaded_20261029_160451_416563';r=json.loads((F/'result.json').read_bytes());p=AdaptiveYawLoadedPlan(F/'fresh_off_request.json')
rows=p.plan(p.source_position());w=[e['requested'] for e in r['write_events'] if e['address']==116];proof=json.loads((F/'current_start_screen.json').read_bytes())
print(json.dumps(dict(rebuilt_goals=len(rows),written_goals=len(w),start=p.source_position().tolist(),proof_start=proof['start_ticks'],
    reused=p.review_center_reused,rebuilt_digest=p.plan_digest(p.source_position()),proof_digest=proof['plan_sha256'],start_difference=r['start_difference_ticks'],enable_shift=r['enable_goal_shift_ticks'])))
tail=w[-len(rows):];bad=[]
for i,((phase,q),v) in enumerate(zip(rows,tail)):
    if q.tolist()!=v:bad.append(dict(index=i,phase=phase,expected=q.tolist(),actual=v))
print(json.dumps(dict(mismatches=len(bad),first=bad[:3],last=bad[-2:])))
