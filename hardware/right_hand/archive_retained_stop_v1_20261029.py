"""Preserve source versions actually used by the stopped run before changes."""
from pathlib import Path
import json,hashlib
ROOT=Path(__file__).resolve().parent
RUN=ROOT/'retained_slow_yaw_20261029_012108_810204';OUT=RUN/'offline_review/execution_source_v1';OUT.mkdir(parents=True,exist_ok=True)
r=json.loads((RUN/'current_start_screen.json').read_bytes());saved={}
for name in ('combined_quiet_reserve_20261029.py','retained_slow_yaw_plan_20261029.py',
    'retained_slow_yaw_gate_20261029.py','leap_retained_slow_yaw_20261029.py',
    'test_combined_quiet_reserve_20261029.py','test_retained_slow_yaw_operator_20261029.py'):
    p=ROOT/name;data=p.read_bytes();digest=hashlib.sha256(data).hexdigest()
    if name in r['code_hashes']:assert digest==r['code_hashes'][name],name
    target=OUT/name
    if not target.exists():target.write_bytes(data)
    saved[name]=digest
for name in ('request.json','review.json','readiness.json','verification.json'):
    p=ROOT/'right_hand_rotation_20261029/retained_slow_yaw_v1'/name
    target=OUT/name
    if not target.exists():target.write_bytes(p.read_bytes())
(OUT/'source_hashes.json').write_text(json.dumps(saved,indent=2)+'\n',encoding='utf-8')
print('Archived actual stopped-run source and review:',OUT)
