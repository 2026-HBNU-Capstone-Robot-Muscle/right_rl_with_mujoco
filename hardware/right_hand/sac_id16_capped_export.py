"""Export bounded EMPTY diagnostic after independent simulation and preparation checks."""
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    oldpath=ROOT/'sac_reachable_empty_path.json';doc=json.loads(oldpath.read_text())
    sources=dict(doc['sources_sha256']);sources[oldpath.name]=hashlib.sha256(oldpath.read_bytes()).hexdigest()
    for backend in ('native','mjx'):
        path=ROOT/f'sac_id16_cap3238_{backend}_validation.json';r=json.loads(path.read_text())
        if not r['all_passed'] or len(r['results'])!=10 or r.get('id16_goal_cap_ticks')!=3238 or r.get('hypothetical_id16_upper_ticks')!=3254:
            raise ValueError('Bounded-path validation failed')
        if r['source_sha256']!=doc['sources_sha256']['sac_reachable_trace_search_result.json']:raise ValueError('Different source path')
        sources[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
    path=ROOT/'sac_id16_preparation_ordered_result.json';prep=json.loads(path.read_text())
    rows=[r for r in prep['results'] if r['order']=='thumb_first']
    if len(rows)!=10 or not all(r['passed'] for r in rows):raise ValueError('Complete preparation/path failed')
    sources[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
    doc['start_ticks'][15]=3238
    for goal in doc['goals_id_order']:goal[15]=min(goal[15],3238)
    doc.update(sources_sha256=sources,id16_goal_cap_ticks=3238,preparation_order='root_relief_then_thumb_then_other_fingers',
               purpose='EMPTY tracking diagnostic with observed ID16 plateau avoided; not live SAC',
               physical_stall_cause='unknown; 3254 ticks is not a certified mechanical limit')
    out=ROOT/'sac_reachable_empty_path_id16capped.json';out.write_text(json.dumps(doc,indent=2)+'\n')
    print(hashlib.sha256(out.read_bytes()).hexdigest())

if __name__=='__main__':main()
