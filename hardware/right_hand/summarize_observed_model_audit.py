import json
from pathlib import Path
r=json.loads((Path(__file__).resolve().parent/'right_hand_rotation_20261024/observed_model_envelope_audit/result.json').read_bytes())
print(json.dumps(r['summary']))
for row in r['cases']['0.1']['dynamic']:
    print(json.dumps(row))
