"""Export ONE successful simulated SAC goal trace for a clearly labelled EMPTY test."""
import hashlib
import json
from pathlib import Path
import numpy as np
from leap_sac_start_empty import target_pose,SIM_TO_ID
from sequence_flip_checked import TICK,validate_raw
ROOT=Path(__file__).resolve().parent

def main():
    path=ROOT/'sac_forward_support_reference_v1_20k'/'supported_batch_eval_sac_cube55_resets_cube55mm.json'
    report=json.loads(path.read_text())
    if report['status']!='offline_supported_batch_evaluation_complete' or report['diagnostic_cube_side_mm']!=55 or report['control_dt']!=.05:
        raise ValueError('Unexpected source evaluation')
    row=next(r for r in report['results'] if r['offset_index']==0 and r['seed']==0)
    trace=row['trace'];last=trace[-1]
    if not last['success'] or any(t.get('support_fault') or not t.get('finite',True) for t in trace):raise ValueError('Source rollout failed')
    if [t['step'] for t in trace]!=list(range(len(trace))):raise ValueError('Discontinuous source steps')
    raw=[]
    for step in trace:
        ctrl=np.asarray(step['ctrl'],float)
        if ctrl.shape!=(16,) or not np.isfinite(ctrl).all():raise ValueError('Invalid trace ctrl')
        ticks=np.empty(16,int);ticks[SIM_TO_ID-1]=np.rint((ctrl+np.pi)/TICK).astype(int);raw.append(validate_raw(ticks))
    raw=np.array(raw);start=target_pose()
    if not np.array_equal(raw[0],start):raise ValueError('Trace does not begin at tested new start')
    floor=int(np.ceil((np.pi+.65)/TICK))
    if np.min(raw[:,[4,8,12]])<floor-1:raise ValueError('Trace violates root floor beyond quantization')
    delta=np.abs(np.diff(raw,axis=0));maxstep=int(delta.max())
    out=dict(format='leap_sac_simulated_trace_empty_v1',empty_only=True,live_policy=False,cube_allowed=False,
             source=str(path.relative_to(ROOT)),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
             checkpoint=report['checkpoint'],source_offset_index=0,source_seed=0,
             dt_seconds=.05,start_ticks=start.tolist(),goals_id_order=raw[1:].tolist(),
             source_success=True,max_step_ticks=maxstep,max_step_rad=float(maxstep*TICK),
             duration_seconds=(len(raw)-1)*.05,root_floor_ticks=floor-1,
             minimum_ticks_by_id=raw.min(0).tolist(),maximum_ticks_by_id=raw.max(0).tolist(),
             caveat='Fixed replay of simulated SAC commands for EMPTY actuator/path validation. No live inference or cube rotation certification.')
    dest=ROOT/'sac_forward_empty_trace.json';dest.write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({k:v for k,v in out.items() if k not in ('goals_id_order','checkpoint')},indent=2))
    print('TRACE_SHA256',hashlib.sha256(dest.read_bytes()).hexdigest())

if __name__=='__main__':main()
