import json
from pathlib import Path
r=json.loads((Path(__file__).parent/'sac_reference_step_probe_result.json').read_text())
for index in (0,4,19):
    row=r['rows'][index]
    print(json.dumps({k:row[k] for k in ('step','joint_error','native_force','mjx_force','native_contacts','mjx_contacts')},indent=2))
