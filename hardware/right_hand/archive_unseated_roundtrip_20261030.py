"""Preserve the rejected initial-placement study and its exact model source."""
import json,hashlib
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/yaw_roundtrip';source=R/'screen_yaw_roundtrip_model_20261030.py'
snapshot=O/'rejected_unseated_model_source.py'
if snapshot.exists():raise RuntimeError('Refuse to overwrite rejected source snapshot')
snapshot.write_bytes(source.read_bytes());sha=hashlib.sha256(snapshot.read_bytes()).hexdigest();cases=[]
for f in (-15,-10,-5):
 p=O/f'full_loaded_front_{f}/review.json';r=json.loads(p.read_bytes());cases.extend(r['cases'])
report=dict(status='unseated_initial_placement_rejected',model_source_snapshot=snapshot.relative_to(R).as_posix(),model_source_sha256=sha,
    cases=len(cases),passed=sum(c['passed'] for c in cases),failed=[dict(front=c['book_front_mm'],gain=c['gain_scale'],friction=c['friction'],
    stop=c['stop'],initial_placement_probe=c['initial_placement_probe']) for c in cases if not c['passed']],
    original_review_bytes_preserved=True,hardware_execution_allowed=False,devices_opened=False)
(O/'rejected_unseated_summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='failed'}))
