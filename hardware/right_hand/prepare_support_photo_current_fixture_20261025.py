import json
from support_photo_path_plan_20261025 import ROOT,FIXTURE,SupportPhotoPathPlan
FIXTURE.mkdir(exist_ok=True,parents=True)
r=json.loads((ROOT/'quarter_support_photo_20261025_214901_644144/fresh_off_request.json').read_bytes());r['plan_version']=SupportPhotoPathPlan.PLAN_VERSION
(FIXTURE/'request.json').write_text(json.dumps(r,indent=2),encoding='utf-8')
p=SupportPhotoPathPlan(FIXTURE/'request.json');rows=p.plan(p.source_position());print('GOALS',len(rows),'FIXTURE',FIXTURE)
