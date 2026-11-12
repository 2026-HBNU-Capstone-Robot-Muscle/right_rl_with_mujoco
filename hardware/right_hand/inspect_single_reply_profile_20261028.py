import json
from pathlib import Path
p=Path(__file__).resolve().parent/'single_reply_profile_20261028_054436_417956/result.json'
r=json.loads(p.read_bytes());failed=[v for v in r['observations'] if v['comm'] or v['error']]
print(json.dumps(dict(failed_reads=failed,failed_packets=[v for v in r['packet_latencies'] if v['comm']],last_positions=r['snapshots'][-1]['position'])))
