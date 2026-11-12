"""READ-only burst confirmation inside a single 200ms sample deadline."""
import time
from contextlib import contextmanager
import numpy as np

POLICY=dict(version='horizontal_bounded_recovery_episode_v1_20261028',
    sample_budget_s=.2,required_consecutive_clean_reads=2,maximum_confirmation_reads=5,
    new_recovered_failure_resets_clean_count=True,recovered_packet_counts_as_clean=False,
    existing_transport_retry_rate_limit_preserved=True,goal_or_setting_writes=0)

@contextmanager
def sample_deadline(bus,began):
    missing=object();previous=getattr(bus,'_horizontal_sample_deadline',missing)
    deadline=began+POLICY['sample_budget_s']
    if previous is not missing:deadline=min(deadline,previous)
    bus._horizontal_sample_deadline=deadline
    try:yield deadline
    finally:
        if previous is missing:del bus._horizontal_sample_deadline
        else:bus._horizontal_sample_deadline=previous

def confirm_recovered_read(bus,command,state,errors,report,phase,*,began,
                           read_sample,validate,now=time.monotonic):
    command=np.asarray(command).copy()
    event=dict(version=POLICY['version'],policy=dict(POLICY),phase=phase,status='confirming',
        command=command.tolist(),goals_advanced=0,goal_writes=0,clean_reads=0,
        required_clean_reads=2,sample_budget_ms=200,sample_indices=[],
        consecutive_clean_sample_indices=[],confirmation_reads=0,clean_count_resets=0)
    report.setdefault('read_recovery_confirmation_events',[]).append(event)
    def within_deadline():
        deadline=min(began+.2,getattr(bus,'_horizontal_sample_deadline',began+.2))
        if now()>=deadline:raise RuntimeError('Recovered read confirmation exceeded the existing 200ms sample budget; OFF')
    def check(fresh):
        validate(fresh)
        if not np.array_equal(fresh['goal'],command):raise RuntimeError('Goal changed during read recovery confirmation; OFF')
    try:
        check(state);within_deadline()
        for _ in range(POLICY['maximum_confirmation_reads']):
            getattr(bus,'cancel_callback',lambda:None)();within_deadline()
            before=len(report.get('read_failures',[]));state,errors=read_sample()
            event['confirmation_reads']+=1
            # Every returned snapshot still passes the unchanged motor checks.
            check(state);within_deadline()
            new=report.get('read_failures',[])[before:]
            if new and not all(f.get('recovered') is True for f in new):
                raise RuntimeError('Unrecovered communication failure; OFF')
            if report.get('samples'):
                index=len(report['samples'])-1
                report['samples'][index]['sample_role']='post_read_recovery_confirmation'
                event['sample_indices'].append(index)
            else:index=None
            if new:
                # A recovered but interrupted READ is not a clean confirmation.
                event['clean_reads']=0;event['consecutive_clean_sample_indices']=[]
                event['clean_count_resets']+=1
                continue
            event['clean_reads']+=1
            if index is not None:event['consecutive_clean_sample_indices'].append(index)
            if event['clean_reads']>=2:
                event.update(status='confirmed',elapsed_ms=(now()-began)*1000)
                return state,errors
        raise RuntimeError('Two consecutive clean snapshots not obtained within confirmation read limit; OFF')
    except (Exception,KeyboardInterrupt) as exc:
        event.update(status='stopped',elapsed_ms=(now()-began)*1000,error=str(exc) or 'Interrupted')
        if isinstance(exc,KeyboardInterrupt):raise
        raise RuntimeError('Fresh-state confirmation after read recovery failed; no target advance: '+str(exc)) from exc
