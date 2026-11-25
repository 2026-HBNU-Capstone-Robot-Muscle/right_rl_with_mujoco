"""READ-only pause after a transport timeout; resume only at the same goal.

No devices are opened by import. The ordinary 200ms sampler and original
motion remain intact. A separate, bounded pause never writes motor registers.
"""
from contextlib import contextmanager
import time

import numpy as np

import calibrated_two_full_health_20261028 as calibrated
import leap_measured_grip_hold_20261117 as pose
import sequence_flip_checked as checked
from grip_transport_recovery_20261125 import RecoveryBus

FAST_SAMPLE = calibrated.sample
POLICY = dict(version='same_goal_READ_only_pause_resume_v1_20261125',
    maximum_extra_pause_s=1., per_read_budget_s=.2, clean_confirmation_window_s=.2,
    required_consecutive_healthy_full_states=2, maximum_requests=24,
    resume_tracking_rad=.12, maximum_pose_drift_rad=.12,
    recovery_probe_methods=['paired_sync', 'full16_unicast'],
    pause_register_writes=0, goal_write_retries=0, automatic_torque_enable=False,
    watchdog_clear_or_extension=False, port_reopen=False, partial_state_reused=False,
    path_index_skips=0, original_fast_read_retry_limit_preserved=True)


class LinkReadFailure(RuntimeError):
    """A transport-only failure, never a motor-health or operator stop."""


class PauseBus(RecoveryBus):
    def __init__(self, report, cancel, **kwargs):
        self._motion_paused = False
        super().__init__(report, cancel, **kwargs)
        report['communication_pause_policy'] = dict(POLICY)
        original_write = self.port.writePort

        def guarded(data):
            if self._motion_paused and (len(data) < 8 or data[7] not in (1, 2, 0x82)):
                raise RuntimeError('Register write prohibited during communication pause')
            return original_write(data)

        self.port.writePort = guarded

    def write(self, address, size, values):
        if self._motion_paused:
            raise RuntimeError('Register write prohibited during communication pause')
        return super().write(address, size, values)

    def snapshot(self):
        before = len(self.report.get('read_failures', []))
        try:
            return super().snapshot()
        except RuntimeError as exc:
            failures = self.report.get('read_failures', [])[before:]
            transport_only = bool(failures) and all(
                f.get('comm', f.get('communication')) in (-3001, -3002)
                and not f.get('device_error') for f in failures)
            from_receiver = (str(exc).startswith('Full16 health recovery:')
                             or str(exc).startswith('Shared200ms expired during READ resynchronization'))
            admitted = self.controlled and 'accepted_start_goal' in self.report
            if admitted and not self._shutdown_started and transport_only and from_receiver:
                raise LinkReadFailure(str(exc)) from exc
            raise

    def pause_probe(self, method, deadline):
        self._using_unicast = method == 'full16_unicast'
        with read_deadline(self, deadline):
            return self._motion_operation()


@contextmanager
def read_deadline(bus, deadline):
    missing = object()
    previous = getattr(bus, '_horizontal_sample_deadline', missing)
    bus._horizontal_sample_deadline = min(deadline,
        previous if previous is not missing else float('inf'))
    try:
        yield
    finally:
        if previous is missing:
            del bus._horizontal_sample_deadline
        else:
            bus._horizontal_sample_deadline = previous


def caused_by_link_failure(exc):
    seen = set()
    while exc is not None and id(exc) not in seen:
        if isinstance(exc, LinkReadFailure):
            return True
        seen.add(id(exc))
        exc = exc.__cause__
    return False


def resume_health(bus, command, report, phase, cause, *, snapshot_check=None,
                  now=time.perf_counter, sleep=time.sleep):
    """No goal/configuration writes, retry-ledger reset, or automatic re-enable."""
    if bus.read_only or bus._shutdown_started or bus._motion_paused:
        raise RuntimeError('Communication pause unavailable in current bus state') from cause
    command = pose.encoder_vector(command).copy()
    previous = getattr(bus, 'last_reviewed_state', None)
    if previous is None:
        raise RuntimeError('No reviewed pose to bound automatic resume') from cause
    anchor = pose.encoder_vector(previous['position']).copy()
    began = now()
    deadline = began + POLICY['maximum_extra_pause_s']
    completed = report.get('completed_goals', 0)
    writes_before = len(report.get('writes', []))
    event = dict(status='waiting', phase=phase, cause=str(cause),
        t=time.monotonic()-report['enabled_at'], command=command.tolist(),
        completed_goals_before=completed, goal_writes=0, register_writes=0,
        clean_reads=0, attempts=[], sample_indices=[], maximum_pause_s=1.)
    report.setdefault('communication_pause_events', []).append(event)
    print('통신 응답 대기: 다음 목표를 멈추고 최대1초 READ만 재시도합니다. SPACE/Q/Esc: 중단.', flush=True)
    bus._motion_paused = True
    clean = 0
    confirmation_began = None
    try:
        for attempt in range(POLICY['maximum_requests']):
            bus.cancel_callback()
            if now() >= deadline:
                break
            at = now()
            row = dict(attempt=attempt+1,
                method=POLICY['recovery_probe_methods'][attempt % 2], quiet={})
            event['attempts'].append(row)
            from combined_quiet_reserve_20261029 import quiet
            quiet(bus, deadline, row['quiet'], now=now, sleep=sleep)
            bus.cancel_callback()
            read_began = now()
            read_until = min(deadline, read_began+POLICY['per_read_budget_s'])
            state, comm, error, details = bus.pause_probe(row['method'], read_until)
            stamp = now()
            row.update(communication=int(comm), device_error=int(error), details=details,
                       elapsed_ms=(stamp-at)*1000)
            if error or comm not in (0, -3001, -3002):
                raise RuntimeError(f'Pause READ device/transport error: comm={comm}, error={error}')
            if comm or stamp >= read_until:
                clean = 0
                confirmation_began = None
                row['accepted'] = False
                event['clean_reads'] = 0
                continue
            if state is None or any(np.asarray(state.get(name)).shape != (16,)
                                    for name, _, _ in pose.FIELDS):
                raise RuntimeError('Pause READ incomplete16 state; no target advance')
            # Any health/goal/watchdog/held-joint error stops immediately. No
            # blanket RuntimeError catch can turn those into transport retries.
            errors, current = checked.health(state, command, **bus.checked_control_profile)
            if snapshot_check is not None:
                snapshot_check(state)
            if np.max(errors) > POLICY['resume_tracking_rad']:
                raise RuntimeError('Resume refused: current-goal tracking exceeds0.12rad')
            if np.max(abs(state['position']-anchor))*checked.TICK > POLICY['maximum_pose_drift_rad']:
                raise RuntimeError('Resume refused: measured pose drift exceeds0.12rad')
            if (report.get('completed_goals', 0) != completed
                    or len(report.get('writes', [])) != writes_before
                    or not np.array_equal(bus.last_goal_command, command)):
                raise RuntimeError('Command/index/register writes changed during READ-only pause')
            if confirmation_began is None or stamp-confirmation_began >= .2:
                confirmation_began = read_began
                clean = 0
            clean += 1
            row['accepted'] = True
            event['clean_reads'] = clean
            report.setdefault('samples', []).append(dict(phase=phase,
                t=time.monotonic()-report['enabled_at'], goal=command.tolist(),
                goal_readback=state['goal'].tolist(), actual=state['position'].tolist(),
                current_ma=current.tolist(), voltage_v=(state['voltage']/10).tolist(),
                temperature_c=state['temperature'].tolist(), tracking_rad=errors.tolist(),
                watchdog=state['watchdog'].tolist(), torque=state['torque'].tolist(),
                error=state['error'].tolist(), read_ms=(stamp-read_began)*1000,
                sample_role='communication_pause_confirmation'))
            event['sample_indices'].append(len(report['samples'])-1)
            if clean >= POLICY['required_consecutive_healthy_full_states']:
                bus.cancel_callback()
                if now() >= deadline or now()-confirmation_began >= .2:
                    raise RuntimeError('Resume confirmation deadline exceeded')
                bus.last_reviewed_state = state
                bus._using_unicast = True
                event.update(status='resumed', elapsed_ms=(now()-began)*1000,
                             completed_goals_after=completed)
                print('통신 복구 확인: 전체16개 정상 상태2회. 같은 목표에서 이어갑니다.', flush=True)
                return state, errors
        raise RuntimeError('Communication pause not recovered within1s/24requests; no target advance') from cause
    except BaseException as exc:
        event.update(status='stopped', elapsed_ms=(now()-began)*1000, error=str(exc),
                     completed_goals_after=report.get('completed_goals', 0))
        raise
    finally:
        bus._motion_paused = False


def sample(bus, command, report, phase, **kwargs):
    try:
        return FAST_SAMPLE(bus, command, report, phase, **kwargs)
    except Exception as exc:
        if not isinstance(bus, PauseBus) or not caused_by_link_failure(exc):
            raise
        state, errors = resume_health(bus, command, report, phase, exc, **kwargs)
    if hasattr(bus, 'grasp_protocol'):
        from new_pair_grasp_plan_20261027 import HOLD
        from horizontal_quarter_plan_20261027 import YAW_HOLD
        if phase == HOLD:
            bus.grasp_protocol.hold_frame(bus, command, report, state)
        if phase == YAW_HOLD:
            bus.grasp_protocol.rotation_frame(bus, command, report, state)
    return state, errors
