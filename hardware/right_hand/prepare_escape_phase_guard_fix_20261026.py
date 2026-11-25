"""Keep the passed escape waypoints/goals; only avoid live full-plan rebuilding."""
import json
from thumb_clear_root_empty_plan_20261026 import ROOT,FIXTURE,ThumbRootEmptyPlan
def main():
    previous=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454/thumb_root_empty_fixture_v9_confirmed_escape'
    proof=json.loads((previous/'screen.json').read_bytes());request=json.loads((previous/'request.json').read_bytes())
    request['plan_version']=ThumbRootEmptyPlan.PLAN_VERSION;FIXTURE.mkdir(exist_ok=True)
    path=FIXTURE/'request.json';path.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    p=ThumbRootEmptyPlan(path)
    if p.plan_digest(p.source_position())!=proof['plan_sha256']:raise ValueError('Accepted escape goal sequence changed')
    print(json.dumps(dict(same_goal_digest=proof['plan_sha256'],goals=len(p.plan(p.source_position())),no_live_plan_rebuild_for_release_indices=True)))
if __name__=='__main__':main()
