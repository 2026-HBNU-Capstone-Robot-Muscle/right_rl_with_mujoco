"""Require fresh healthy packets before advancing after a recovered read.

No setting or goal writes, no old samples reused. The whole sample operation
keeps the existing 200ms bound, including its initial read and any recovery.
"""
import time
import numpy as np

VERSION='two_clean_snapshots_after_recovered_read_v1'
CONFIRM_READS=2
SAMPLE_BUDGET_S=.2


def confirm_recovered_read(bus,command,state,errors,report,phase,*,began,
                           read_sample,validate,now=time.monotonic):
    command=np.asarray(command).copy()
    event=dict(version=VERSION,phase=phase,status='confirming',
        command=command.tolist(),goals_advanced=0,goal_writes=0,
        clean_reads=0,required_clean_reads=CONFIRM_READS,
        sample_budget_ms=SAMPLE_BUDGET_S*1000,sample_indices=[])
    report.setdefault('read_recovery_confirmation_events',[]).append(event)
    try:
        if not np.array_equal(state['goal'],command):
            raise RuntimeError('Recovered read did not contain the exact command; OFF')
        validate(state)
        for _ in range(CONFIRM_READS):
            if now()-began>=SAMPLE_BUDGET_S:
                raise RuntimeError('Recovered read confirmation exceeded the existing 200ms sample budget; OFF')
            failures=len(report.get('read_failures',[]))
            state,errors=read_sample()
            if len(report.get('read_failures',[]))!=failures:
                raise RuntimeError('Another communication failure during read recovery confirmation; OFF')
            validate(state)
            if not np.array_equal(state['goal'],command):
                raise RuntimeError('Goal changed during read recovery confirmation; OFF')
            if now()-began>=SAMPLE_BUDGET_S:
                raise RuntimeError('Recovered read confirmation exceeded the existing 200ms sample budget; OFF')
            event['clean_reads']+=1
            if report.get('samples'):
                index=len(report['samples'])-1
                report['samples'][index]['sample_role']='post_read_recovery_confirmation'
                event['sample_indices'].append(index)
        event.update(status='confirmed',elapsed_ms=(now()-began)*1000)
        return state,errors
    except (Exception,KeyboardInterrupt) as exc:
        event.update(status='stopped',elapsed_ms=(now()-began)*1000,error=str(exc) or 'Interrupted')
        if isinstance(exc,KeyboardInterrupt):raise
        raise RuntimeError('Fresh-state confirmation after read recovery failed; no target advance: '+
                           (str(exc) or 'Interrupted')) from exc
