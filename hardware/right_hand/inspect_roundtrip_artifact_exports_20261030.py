import json
from pathlib import Path
R=Path(__file__).resolve().parent
for f in sorted(R.glob('yaw_roundtrip_trial_*/offline_review/video_exports.json')):
 print(f.relative_to(R),f.read_text(encoding='utf-8'));print([(p.name,p.stat().st_size) for p in f.parent.glob('*.mp4')])
f=R/'fast_off_readonly_20261030_042645_734940/result.json';r=json.loads(f.read_bytes());print('RECENT_OFF',r.get('status'),r.get('stop'),r.get('servo_writes'),r.get('torque_changes'))
