"""Observe the existing successful READ payload; no additional I/O or recovery writes."""
import time

from grip_pause_resume_10s_trial_20261125 import PauseBus as BasePauseBus


class MotorStateObserver:
    def __init__(self, report):
        self.report = report
        self.previous = {}
        self.summary = dict(version='existing_CRC_checked_READ_clock_RAM_observer_v1',
            additional_motor_reads=0, additional_motor_writes=0,
            observed_packets=0, transitions=[], clock_discontinuities=[])
        report['motor_state_observer'] = self.summary

    def observe(self, motor, data, stamp, *, active):
        if not 1 <= motor <= 16 or len(data) != 83:
            return

        def value(address, length):
            at = address-64
            return int.from_bytes(bytes(data[at:at+length]), 'little')

        row = dict(motor_id=motor, qpc_seconds=stamp, torque=value(64,1),
            error=value(70,1), d_gain=value(80,2), i_gain=value(82,2),
            p_gain=value(84,2), watchdog=value(98,1), current_cap=value(102,2),
            goal=value(116,4), realtime_tick=value(120,2),
            position=value(132,4), voltage_v=value(144,2)/10,
            temperature_c=value(146,1))
        before = self.previous.get(motor)
        self.previous[motor] = row
        self.summary['observed_packets'] += 1
        self.summary['latest_per_motor'] = [self.previous[k] for k in sorted(self.previous)]
        if not active or before is None:
            return
        elapsed = stamp-before['qpc_seconds']
        # Counter rollover alone is normal. A discrepancy is evidence to
        # correlate with RAM changes, not proof of a reboot or faulty motor.
        discontinuity = False
        if (0 <= elapsed < 20. and 0 <= before['realtime_tick'] < 32768
                and 0 <= row['realtime_tick'] < 32768):
            expected = (before['realtime_tick']+round(elapsed*1000)) % 32768
            residual = (row['realtime_tick']-expected+16384) % 32768-16384
            if abs(residual) > 250:
                discontinuity = True
                if len(self.summary['clock_discontinuities']) < 64:
                    self.summary['clock_discontinuities'].append(dict(
                        before=before, after=row, elapsed_ms=elapsed*1000,
                        expected_tick=expected, counter_residual_ms=residual,
                        reset_not_confirmed=True))
        changed = ((before['torque']==1 and row['torque']==0)
                   or (before['watchdog']==25 and row['watchdog']!=25)
                   or before['current_cap'] != row['current_cap']
                   or any(before[k] != row[k] for k in ('d_gain','i_gain','p_gain')))
        if changed and len(self.summary['transitions']) < 64:
            self.summary['transitions'].append(dict(before=before, after=row,
                clock_discontinuity=discontinuity, payload_hex=bytes(data).hex(),
                reset_or_external_write_not_yet_distinguished=True))


class ObservedPauseBus(BasePauseBus):
    def __init__(self, report, cancel, **kwargs):
        super().__init__(report, cancel, **kwargs)
        self.state_observer = MotorStateObserver(report)
        original = self.packet.readRx

        def observed(port, motor, length):
            result = original(port, motor, length)
            data, comm, error = result
            if port is self.port and length == 83 and not comm and not error and len(data)==83:
                try:
                    active = (self.controlled and 'accepted_start_goal' in report
                              and not self._shutdown_started)
                    self.state_observer.observe(motor, data, time.perf_counter(), active=active)
                except Exception:
                    summary = self.state_observer.summary
                    summary['observation_errors'] = summary.get('observation_errors', 0)+1
            return result  # Keep SDK result, errors and original health checks unchanged.

        self.packet.readRx = observed

    def _motion_operation(self):
        result = super()._motion_operation()
        state, comm, error, details = result
        if (self.controlled and 'accepted_start_goal' in self.report
                and not self._shutdown_started and not comm and not error and state is not None):
            bad = [i+1 for i in range(16) if state['torque'][i] != 1
                   or state['watchdog'][i] != 25 or state['error'][i]]
            if bad:
                self.report.setdefault('first_unexpected_motor_state', dict(
                    phase=self.report.get('phase'), completed_goals=self.report.get('completed_goals'),
                    qpc_seconds=time.perf_counter(), motor_ids=bad,
                    full_state={k:v.tolist() for k,v in state.items()},
                    observed_before_shutdown_OFF=True))
        return result
