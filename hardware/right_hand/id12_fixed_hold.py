"""One bounded fixed-goal observation; READ only while already enabled."""
import time
import numpy as np
import sequence_flip_checked as checked
from id12_hold_spec import HOLD_SECONDS

def observe(bus,command,report,*,now=time.monotonic,sleep=time.sleep,
            report_key='fixed_hold',phase='id12_fixed_goal_hold',next_action='OFF'):
    report['phase']=phase
    began=now();positions=[];currents=[]
    report[report_key]=dict(goal=command.tolist(),requested_seconds=HOLD_SECONDS,
        goal_writes=0,completed=False)
    print(f'HOLD DIAGNOSTIC: all goals fixed for 1s; readings only, then {next_action}. Q stops.',flush=True)
    for _ in range(32):
        checked.check_cancel()
        if positions and now()-began>=HOLD_SECONDS:break
        state,_=checked.sample(bus,command,report,report['phase'])
        positions.append(np.asarray(state['position']).copy())
        currents.append(checked.signed_current(np.asarray(state['current'])))
        remaining=HOLD_SECONDS-(now()-began)
        if remaining>0:sleep(min(.05,remaining))
    elapsed=now()-began
    if elapsed<HOLD_SECONDS or len(positions)<2:
        raise RuntimeError('Fixed hold did not obtain enough fresh samples; no continuation')
    actual=np.array(positions);initial=actual[0];tail=np.median(actual[-5:],axis=0)
    before=np.abs(command-initial)*checked.TICK;after=np.abs(command-tail)*checked.TICK
    report[report_key].update(completed=True,elapsed_seconds=elapsed,reads=len(positions),
        actual_start=initial.tolist(),actual_end_median=tail.tolist(),
        tracking_start_rad=before.tolist(),tracking_end_median_rad=after.tolist(),
        current_end_median_ma=np.median(currents[-5:],axis=0).tolist())
    print(f'ID12 fixed goal {command[11]}: actual {initial[11]} -> {tail[11]:.1f}; '
          f'error {before[11]:.3f} -> {after[11]:.3f}rad. Next: {next_action}.',flush=True)
