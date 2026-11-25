"""Bounded read-only confirmation of an exact previous goal packet.

A mismatching register is never accepted as the command, and a goal is never
resent. Only the screened runner opts in after its strict enable checks.
"""
import time
import numpy as np

CONFIRM_SECONDS = .15
MAX_READS = 3
EXACT_READS = 2
MAX_STEP_TICKS = 2
REPEAT_WINDOW_SECONDS = 10.
REPEAT_TRIP = 3


class GoalReadbackMismatch(RuntimeError):
    pass


def confirm_previous_goal(bus, command, initial, report, phase, *, read_sample, validate):
    command = np.asarray(command).copy()
    previous = getattr(bus, 'previous_goal_command', None)
    last = getattr(bus, 'last_goal_command', None)
    began = time.monotonic()
    event = dict(phase=phase, t=began-report['enabled_at'], status='confirming',
                 command=command.tolist(), initial_readback=np.asarray(initial['goal']).tolist(),
                 previous_command=None if previous is None else np.asarray(previous).tolist(),
                 confirmation_reads=0, goals_advanced=0, goal_writes=0, observations=[])
    events = report.setdefault('goal_readback_events', [])
    events.append(event)

    def candidate(goal):
        a = np.asarray(goal)
        return (a.shape == (16,) and np.isfinite(a).all()
                and np.all(a == np.rint(a)) and np.all((0 <= a) & (a <= 4095)))

    try:
        # The whole observed vector must equal the immediately preceding
        # issued packet. Arbitrary small errors/partial writes do not qualify.
        if (not candidate(command) or not candidate(previous) or not candidate(last)
                or not candidate(initial['goal']) or not np.array_equal(last, command)
                or not np.array_equal(initial['goal'], previous)
                or np.array_equal(previous, command)
                or np.max(np.abs(np.asarray(previous)-command)) > MAX_STEP_TICKS):
            raise RuntimeError('Goal readback is not the exact preceding <=2-tick packet; no target advance')
        if sum(0 <= event['t']-e['t'] <= REPEAT_WINDOW_SECONDS for e in events) >= REPEAT_TRIP:
            raise RuntimeError('Repeated goal readback mismatch: 3 events within 10s; OFF')
        print('Goal readback: holding the same command; fresh-read confirmation, no goal resend.', flush=True)
        good = 0
        for _ in range(MAX_READS):
            if time.monotonic()-began >= CONFIRM_SECONDS:
                break
            # read_sample also enforces Q, read time and per-snapshot joint guards.
            state, errors = read_sample()
            event['confirmation_reads'] += 1
            event['observations'].append(np.asarray(state['goal']).tolist())
            try:
                validate(state)
            except GoalReadbackMismatch:
                if not np.array_equal(state['goal'], previous):
                    raise RuntimeError('Goal readback changed outside the preceding packet; OFF')
                good = 0
            else:
                good += 1
            elapsed = time.monotonic()-began
            if elapsed >= CONFIRM_SECONDS:
                break
            if good >= EXACT_READS:
                event.update(status='recovered', elapsed_ms=elapsed*1000)
                print('Goal readback matched the exact command twice; continuing from the same step.', flush=True)
                return state, errors
            time.sleep(.01)
        raise RuntimeError('Goal readback did not match the exact command twice within 150ms/3 reads; OFF')
    except (Exception, KeyboardInterrupt) as exc:
        event.update(status='stopped', elapsed_ms=(time.monotonic()-began)*1000,
                     error=str(exc) or 'Interrupted')
        raise
