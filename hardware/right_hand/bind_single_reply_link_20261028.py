"""Replace only host-link comparison evidence; no motor setting changes."""
import hashlib,json
from horizontal_quarter_plan_20261027 import ROOT,OUT
cfg=ROOT/'hand_serial_link_20261028.json';c=json.loads(cfg.read_bytes())
source=ROOT/'sync_burst_readonly_20261028_043357_661811/result.json';r=json.loads(source.read_bytes())
assert r['blocks'][0]['group_size']==1 and r['blocks'][0]['complete']==1000 and not r['blocks'][0]['errors'] and not r['driver_errors']
c.update(readonly_check=source.relative_to(ROOT).as_posix(),readonly_check_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    eight_reply_production_check_failed='production_link_readonly_20261028_042900_797572/result.json',
    full_health_reply_group_size=1)
cfg.write_bytes((json.dumps(c,indent=2)+'\n').encode())
if (OUT/'review.json').exists() and not (OUT/'review_eight_reply_diagnostic.json').exists():
    (OUT/'review_eight_reply_diagnostic.json').write_bytes((OUT/'review.json').read_bytes())
from horizontal_serial_link_20261028 import metadata
print(json.dumps(metadata()))
