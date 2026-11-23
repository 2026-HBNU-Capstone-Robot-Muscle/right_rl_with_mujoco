"""Activate the passing20:04 start review, preserving the previous proof."""
import json,shutil
from pathlib import Path
import cube_grasp_hold_20261116 as c
folder=c.OUT/'start_fix_200443'
raw=json.loads((folder/'request.json').read_bytes())
empty=json.loads((folder/'cube_empty_recovery/review.json').read_bytes())
if not empty['all_passed']:raise RuntimeError('Current start has not passed; keep old proof')
backup=folder/'previous_active';backup.mkdir(exist_ok=True)
for p in (c.REQUEST,c.PROOF):
    dest=backup/p.name
    if not dest.exists():shutil.copyfile(p,dest)
shutil.copyfile(folder/'request.json',c.REQUEST)
p=c.Plan(c.REQUEST);r=c.make_review(p,empty)
c.PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
c.validate_review(p,c.PROOF)
print(json.dumps(dict(status=r['status'],start=p.source_position().tolist(),
    goals=len(p.plan(p.source_position())),static=sum(x['passed'] for x in empty['static_cases']),
    dynamic=sum(x['passed'] for x in empty['dynamic_cases']),devices_opened=False)))
