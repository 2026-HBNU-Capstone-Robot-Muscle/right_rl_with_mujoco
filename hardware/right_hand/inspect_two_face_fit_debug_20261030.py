import json
from fit_two_face_placement_20261030 import R,F,O
r=json.loads((O/'two_face_placement_comparison.json').read_bytes())
for name,g in r['groups'].items():
 row=g['rows'][0];print(name,json.dumps(row['fit']['attempts']))
