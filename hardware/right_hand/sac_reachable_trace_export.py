"""Export a pinned EMPTY-only actuator diagnostic after independent simulations."""
import hashlib
import json
from pathlib import Path
import numpy as np
from leap_sac_start_empty import target_pose, SIM_TO_ID
from sequence_flip_checked import TICK, validate_raw
ROOT=Path(__file__).resolve().parent

def main():
    source=ROOT/'sac_reachable_trace_search_result.json';content=source.read_bytes();digest=hashlib.sha256(content).hexdigest()
    row=next(r for r in json.loads(content)['results'] if r['compensation_rad']==.07 and r['time_scale']==1.)
    if not row['passed']:raise ValueError('Source search failed')
    provenance={source.name:digest}
    for backend in ('native','mjx'):
        path=ROOT/f'sac_reachable_trace_{backend}_validation.json';data=path.read_bytes();report=json.loads(data)
        if (not report['all_passed'] or len(report['results'])!=10 or report['source_sha256']!=digest
                or report['compensation_rad']!=.07 or report['time_scale']!=1. or not report['quantized_ticks']):
            raise ValueError(f'{backend} validation incomplete or failed')
        provenance[path.name]=hashlib.sha256(data).hexdigest()
    raw=np.empty((len(row['goals_sim_order']),16),int)
    raw[:,SIM_TO_ID-1]=np.rint((np.array(row['goals_sim_order'])+np.pi)/TICK).astype(int)
    for r in raw:validate_raw(r)
    if not np.array_equal(raw[0],target_pose()):raise ValueError('Start mismatch')
    raw=np.r_[raw,np.repeat(raw[-1:],30,axis=0)]
    doc=dict(format='leap_reachable_empty_path_v1',empty_only=True,live_policy=False,cube_allowed=False,
             purpose='Measure hardware tracking/timing along a refined SAC-derived path; not a live SAC trial',
             sources_sha256=provenance,dt_seconds=.05,extra_hold_steps=30,start_ticks=raw[0].tolist(),
             goals_id_order=raw[1:].tolist(),max_step_ticks=int(np.abs(np.diff(raw,axis=0)).max()),
             duration_seconds=(len(raw)-1)*.05,root_floor_ticks=int(np.floor((np.pi+.65)/TICK)))
    out=ROOT/'sac_reachable_empty_path.json';out.write_text(json.dumps(doc,indent=2)+'\n')
    print('PATH_SHA256',hashlib.sha256(out.read_bytes()).hexdigest())
    print('goals',len(raw)-1,'seconds',doc['duration_seconds'],'max_step_ticks',doc['max_step_ticks'])

if __name__=='__main__':main()
