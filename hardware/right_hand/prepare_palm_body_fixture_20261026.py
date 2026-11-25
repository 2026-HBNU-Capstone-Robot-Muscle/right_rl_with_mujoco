"""Use the actual failed attempt as a device-free model-regression input."""
import json
from thumb_clear_root_empty_plan_20261026 import ROOT,FIXTURE,ThumbRootEmptyPlan
def main():
    source=ROOT/'thumb_clear_root_empty_20261026_123524_339920/fresh_off_request.json'
    r=json.loads(source.read_bytes());r['plan_version']=ThumbRootEmptyPlan.PLAN_VERSION
    FIXTURE.mkdir(exist_ok=True);path=FIXTURE/'request.json';path.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
    p=ThumbRootEmptyPlan(path);print(json.dumps(dict(start=p.source_position().tolist(),goals=len(p.plan(p.source_position())),motor_writes=0)))
if __name__=='__main__':main()
