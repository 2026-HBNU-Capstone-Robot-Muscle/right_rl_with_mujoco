"""READ-only confirmation of per-servo current/previous packet observations.

No tolerance is applied to accepted goals: two whole fresh snapshots must match
the current command exactly before the caller may advance. No write API is used.
"""
import time
import numpy as np
from goal_readback_confirmation import GoalReadbackMismatch

POLICY = dict(version='horizontal_per_servo_packet_confirmation_v1_20261027',
    confirmation_seconds=.15, maximum_reads=3, exact_current_reads=2,
    maximum_packet_step_ticks=2, pending_values='exact_current_or_immediate_previous_per_motor',
    accepted_values='exact_current_whole_vector', goal_writes=0,
    repeat_window_seconds=10., repeat_trip=3)


def vector(value):
    a=np.asarray(value)
    return (a.shape==(16,) and np.isfinite(a).all() and
            np.all(a==np.rint(a)) and np.all((0<=a)&(a<=4095)))


def confirm_previous_goal(bus,command,initial,report,phase,*,read_sample,validate):
    command=np.asarray(command).copy();previous=getattr(bus,'previous_goal_command',None)
    last=getattr(bus,'last_goal_command',None);began=time.perf_counter()
    event=dict(phase=phase,t=time.monotonic()-report['enabled_at'],status='confirming',
        policy=dict(POLICY),command=command.tolist(),initial_readback=np.asarray(initial['goal']).tolist(),
        previous_command=None if previous is None else np.asarray(previous).tolist(),
        confirmation_reads=0,goals_advanced=0,goal_writes=0,observations=[])
    events=report.setdefault('goal_readback_events',[]);events.append(event)

    def pending(goal):
        return vector(goal) and np.all((np.asarray(goal)==command)|(np.asarray(goal)==previous))

    try:
        if (not vector(command) or not vector(previous) or not vector(last)
            or not np.array_equal(last,command) or np.array_equal(previous,command)
            or np.max(abs(np.asarray(previous)-command))>2 or not pending(initial['goal'])):
            raise RuntimeError('Goal readback has an unknown/older value; no target advance')
        if sum(0<=event['t']-e['t']<=10. for e in events)>=3:
            raise RuntimeError('Repeated goal readback mismatch: 3 events within 10s; OFF')
        print('Goal readback pending on one or more IDs: holding goals; READ only, exact current twice within150ms.',flush=True)
        good=0
        for _ in range(3):
            if time.perf_counter()-began>=.15:break
            state,errors=read_sample();event['confirmation_reads']+=1
            event['observations'].append(np.asarray(state['goal']).tolist())
            if not pending(state['goal']):raise RuntimeError('Goal readback left the two known packets; OFF')
            try:validate(state)
            except GoalReadbackMismatch:good=0
            else:
                if not np.array_equal(state['goal'],command):raise RuntimeError('Nonexact goal passed health validation; OFF')
                good+=1
            elapsed=time.perf_counter()-began
            if elapsed>=.15:break
            if good>=2:
                event.update(status='recovered',elapsed_ms=elapsed*1000)
                print('All goals matched the current command exactly in two fresh healthy reads; same queued step.',flush=True)
                return state,errors
            # Full health READ time already exceeds the old 10ms retry interval.
            # Do not insert an extra delay before the second exact confirmation.
        raise RuntimeError('Exact current goals not confirmed twice within150ms/3 reads; OFF')
    except (Exception,KeyboardInterrupt) as exc:
        event.update(status='stopped',elapsed_ms=(time.perf_counter()-began)*1000,error=str(exc) or 'Interrupted')
        raise
