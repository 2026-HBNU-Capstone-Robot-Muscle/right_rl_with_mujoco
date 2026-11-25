from pathlib import Path
import json,numpy as np
R=Path(__file__).resolve().parent;prior=json.loads((R/'fast_off_readonly_20261030_111437_806919/result.json').read_bytes());current=max(R.glob('resume_off_readonly_20261104_*/result.json'),key=lambda p:p.stat().st_mtime);data=json.loads(current.read_bytes());print('SOURCE',current.parent.name);print('VALIDATION',data.get('pose_health_validation_errors'),data.get('error'))
for s in data['snapshots']:
 print(json.dumps({k:s[k] for k in ('position','torque','voltage','temperature')},ensure_ascii=False));delta=np.array(s['position'])-prior['snapshots'][-1]['position'];print('CHANGED_TICKS',[(i+1,int(v)) for i,v in enumerate(delta) if abs(v)>16])
