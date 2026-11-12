import json
from pathlib import Path
R=Path(__file__).resolve().parent/'right_hand_rotation_20261029'
for f in (R/'observed_inverse_tail').glob('*/proposal.json'):
 r=json.loads(f.read_bytes());s=r['frames'][-1];print(json.dumps(dict(name=r['name'],steps=len(r['frames']),last=s)),flush=True)
r=json.loads((R/'observed_inverse_settling/review.json').read_bytes())
print('SETTLING',json.dumps([dict(gain=c['gain_scale'],mu=c['friction'],passed=c['passed'],last=c['history'][-1]) for c in r['cases'] if c['passed']]),flush=True)
