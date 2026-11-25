"""Bounded READ recovery and independent torque-OFF verification.

The original motion, goal validator and controller remain unchanged. Imports
open no devices. Shutdown can only issue OFF until all16 motors confirm OFF.
"""
from collections import deque
from time import perf_counter as clock, sleep
import numpy as np

import leap_measured_grip_hold_20261117 as pose

ORIGINAL_BUS = pose.PoseBus
POLICY = dict(version='full_health_unicast_recovery_independent_OFF_v2_uart_observer_20261125',
    motion_period_s=.075, maximum_goal_step_ticks=2, current_limit_mA=450,
    motion_sample_budget_s=.2, motion_maximum_requests=3, motion_retries_per10s=3,
    healthy_full_snapshots_after_recovery=2, partial_snapshots_accepted=False,
    fallback_instruction='individual_READ_full83_all16',
    shutdown_uses_motion_retry_budget=False, shutdown_verification_rounds=2,
    shutdown_read_budget_per_round_s=.2, shutdown_only_writes_before_verified='TorqueEnable64=0',
    goal_write_retries=0, host_write_timeout_s=.2, individual_OFF_loop_budget_s=1.5,
    existing_windows_driver_error_calls_observed_without_new_calls=True)


class RecoveryBus(ORIGINAL_BUS):
    def __init__(self, report, cancel, *, read_only=False):
        super().__init__(report, cancel)
        self.read_only = read_only
        self._using_unicast = False
        self._shutdown_started = self._off_verified = False
        self._read_recovery_times = []
        self._packets = deque(maxlen=48)
        report['communication_repair_policy'] = dict(POLICY)
        report['read_only_mode'] = read_only
        self.port.ser.write_timeout = .2
        original_write = self.port.writePort

        def traced(data):
            instruction = data[7] if len(data) >= 8 else None
            address = int.from_bytes(bytes(data[8:10]), 'little') if instruction in (2, 3, 0x82, 0x83) else None
            if self.read_only and instruction not in (1, 2, 0x82):
                raise RuntimeError('Read-only link test blocked a non-READ instruction')
            if self._shutdown_started and instruction not in (1, 2, 0x82):
                # Direct scalar OFF and original RAM restoration are the only
                # permitted writes after shutdown; no new position or enable.
                if instruction == 3:
                    allowed = address == 64 and len(data) == 13 and data[10] == 0
                elif instruction == 0x83:
                    allowed = self._off_verified and address in (98, 102, 108, 112)
                else:
                    allowed = False
                if not allowed:
                    raise RuntimeError('Write blocked after shutdown began')
            event = dict(instruction=instruction, destination=data[4] if len(data) > 4 else None,
                         address=address, bytes=len(data), raw_hex=bytes(data).hex())
            self._packets.append(event)
            self.report['last_outgoing_packets'] = list(self._packets)
            sent = original_write(data)
            event['bytes_sent_to_host_driver'] = sent
            return sent

        self.port.writePort = traced
        try:
            from serial_error_observer_20261125 import attach
            self._driver_error_observer = attach(self.port.ser, report)
        except BaseException:
            super().close()
            raise

    def close(self):
        try:
            return super().close()
        finally:
            observer = getattr(self, '_driver_error_observer', None)
            if observer is not None:
                observer.close()

    def _read_motor(self, motor, address, length, deadline):
        self.cancel_callback()
        if clock() >= deadline:
            return [], -3001, 0
        comm = self.packet.readTx(self.port, motor, address, length)
        if comm:
            return [], comm, 0
        remaining = min(.040, deadline-clock())
        if remaining <= 0:
            self.port.is_using = False
            return [], -3001, 0
        self.port.setPacketTimeoutMillis(remaining*1000)
        data, comm, error = self.packet.readRx(self.port, motor, length)
        if clock() >= deadline:
            return [], -3001, error
        if not comm and len(data) != length:
            return data, -3002, error
        return data, comm, error

    def _unicast_full_health(self):
        deadline = min(clock()+.2, getattr(self, '_horizontal_sample_deadline', float('inf')))
        values = {name:np.empty(16, dtype=np.int64) for name, _, _ in pose.FIELDS}
        details = dict(read_method='individual_READ_full83_all16', completed_ids=[],
                       first_incomplete_id=None, partial_snapshot_reused=False)
        for motor in range(1, 17):
            data, comm, error = self._read_motor(motor, 64, 83, deadline)
            if comm or error:
                details.update(first_incomplete_id=motor, received_data_bytes=len(data))
                return None, comm, error, details
            for name, address, size in pose.FIELDS:
                values[name][motor-1] = int.from_bytes(bytes(data[address-64:address-64+size]), 'little')
            details['completed_ids'].append(motor)
        return values, 0, 0, details

    def _motion_operation(self):
        from grip_health_read_20261118 import block_once
        if self._using_unicast:
            result = self._unicast_full_health()
        else:
            result = block_once(self, 64, 83, pose.FIELDS)
        state, comm, error, details = result
        if not comm and not error:
            if state is None or any(name not in state or np.asarray(state[name]).shape != (16,)
                                    for name, _, _ in pose.FIELDS):
                raise RuntimeError('Incomplete full16 health state; no target advance')
        elif comm in (-3001, -3002) and not error:
            self._using_unicast = True
            self.report['unicast_full_health_recovery_selected'] = True
        return state, comm, error, details

    def snapshot(self):
        if self._shutdown_started:
            # Restoration verification is independent of the motion ledger.
            state, comm, error, details = self._unicast_full_health()
            if comm or error or state is None:
                raise RuntimeError(f'OFF restoration full16 read failed: comm={comm}, error={error}, details={details}')
            return state
        if self.controlled and getattr(self, '_horizontal_sample_deadline', None) is not None:
            from combined_quiet_reserve_20261029 import receive
            # Return the first new complete snapshot to the unchanged OUTER
            # sampler, which validates TWO fresh complete healthy snapshots.
            return receive(self, self._motion_operation, 'Full16 health recovery',
                           dict(kind='sync_read', address=64, length=83), now=clock, sleep=sleep)
        return super().snapshot()

    def write(self, address, size, values):
        if self.read_only:
            raise RuntimeError('Read-only link test cannot write motor registers')
        if self._shutdown_started:
            if address == 116 or (address == 64 and np.any(values)):
                raise RuntimeError('Goal/enable prohibited after shutdown')
            if address != 64 and not self._off_verified:
                raise RuntimeError('RAM restoration prohibited before all16 OFF confirmed')
        return super().write(address, size, values)

    def _verify_OFF(self):
        began = clock()
        deadline = began+.2
        event = dict(motion_retry_allowance_used=False, verified=False, readings=[],
                     full16_required=True, budget_ms=200)
        self.report.setdefault('independent_shutdown_checks', []).append(event)
        confirmed = set()
        for motor in range(1, 17):
            if clock() >= deadline:
                break
            try:
                data, comm, error = self._read_motor(motor, 64, 1, deadline)
            except Exception as exc:
                event['readings'].append(dict(id=motor, torque=None, error=str(exc)))
                continue
            value = data[0] if not comm and not error and len(data) == 1 else None
            event['readings'].append(dict(id=motor, communication=int(comm), device_error=int(error), torque=value))
            if value == 0:
                confirmed.add(motor)
        unknown = sorted(set(range(1, 17))-confirmed)
        event.update(verified=not unknown, unconfirmed_ids=unknown, elapsed_ms=(clock()-began)*1000)
        return unknown

    def off(self):
        if self.read_only:
            raise RuntimeError('Read-only test cannot send OFF or any other motor write')
        self._shutdown_started = True
        self._off_verified = False
        self.report['all_torque_off_verified'] = False
        # Compact broadcast WRITE64=0: every packet here only turns torque OFF.
        try:
            comm = self.packet.write1ByteTxOnly(self.port, 254, 64, 0)
            broadcast = dict(address=64, value=0, communication=int(comm), verified=False)
        except Exception as exc:
            broadcast = dict(address=64, value=0, error=str(exc), verified=False)
        self.report['compact_broadcast_OFF'] = broadcast
        self.report.setdefault('writes', []).append(dict(address=64, values=[0]*16,
                                                        instruction='compact_broadcast_WRITE'))
        unknown = self._verify_OFF()
        if unknown:
            events = self.report.setdefault('individual_OFF_transmissions', [])
            deadline = clock()+1.5
            for motor in range(1, 17):
                if clock() >= deadline:
                    break
                try:
                    # Wait for the unicast WRITE status before transmitting the
                    # next packet; TxOnly bursts can collide with those replies.
                    result, error = self.packet.write1ByteTxRx(self.port, motor, 64, 0)
                    events.append(dict(id=motor, address=64, value=0,
                                       communication=int(result), device_error=int(error)))
                except Exception as exc:
                    events.append(dict(id=motor, address=64, value=0, error=str(exc)))
            unknown = self._verify_OFF()
        self.report['individual_off_unconfirmed_ids'] = unknown
        if unknown:
            raise RuntimeError('Torque OFF unconfirmed for IDs '+','.join(map(str, unknown))+
                               '; independent shutdown checks failed, motor power OFF required')
        self._off_verified = True
        self.report['all_torque_off_verified'] = True
        self.report['compact_broadcast_OFF']['verified'] = True
