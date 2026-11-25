"""A one-tick synthetic encoder change checks recovery recalculation/cache scope."""
import json
from new_pair_grasp_plan_20261027 import OUT,REQUEST,NewPairGraspPlan,loaded_digest
folder=OUT/'cache_probe';folder.mkdir(exist_ok=True)
r=json.loads(REQUEST.read_bytes());r['off_state']['position'][0]+=1;r['synthetic_fixture']=True;r['hardware_execution_allowed']=False
path=folder/'request.json';path.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
a=NewPairGraspPlan(REQUEST);b=NewPairGraspPlan(path)
assert a.plan_digest(a.source_position())!=b.plan_digest(b.source_position())
assert loaded_digest(a)==loaded_digest(b)
print('Synthetic changed recovery; identical fixed loaded cycle. No devices opened.')
