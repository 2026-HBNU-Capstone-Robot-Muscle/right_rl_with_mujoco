"""Preserve the pre-fix code and canonical goal sequence before optimization."""
import hashlib,json,shutil
from thumb_clear_root_empty_plan_20261026 import ROOT,FIXTURE,FILES,ThumbRootEmptyPlan
def main():
    dest=FIXTURE/'before_timeout_fix';dest.mkdir(exist_ok=True)
    for name in FILES:shutil.copyfile(ROOT/name,dest/name)
    p=ThumbRootEmptyPlan(FIXTURE/'request.json');raw=p.source_position();rows=p.plan(raw)
    result=dict(source_request=str((FIXTURE/'request.json').relative_to(ROOT)),start_ticks=raw.tolist(),
        goal_count=len(rows),plan_digest=p.plan_digest(raw),
        goal_sequence=[(phase,q.tolist()) for phase,q in rows],
        hashes={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES})
    (dest/'basis.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print('Archived',len(rows),'goals',result['plan_digest'])
if __name__=='__main__':main()
