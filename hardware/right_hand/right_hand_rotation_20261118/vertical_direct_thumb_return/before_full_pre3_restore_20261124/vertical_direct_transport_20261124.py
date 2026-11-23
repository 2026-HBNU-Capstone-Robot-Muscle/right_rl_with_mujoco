"""Direct-cycle transport spacing and bounded shutdown-only verification.

Imports open no devices. Motion retains full-health reads, the shared 200ms
deadline and three retries per 10s. Its outer sampler validates both recovery
snapshots, so the transport does not also demand two unvalidated snapshots.
"""
from time import perf_counter as clock, sleep
import numpy as np
import leap_measured_grip_hold_20261117 as pose

HEALTH_READ_GAP_SECONDS=.010
POLICY=dict(version='direct_outer_two_complete_shutdown_verify_v2_20261124',
    minimum_complete_health_read_gap_ms=10,full16_required=True,
    motion_recovery_policy_unchanged=True,motion_sample_budget_ms=200,
    motion_confirmation_owner='calibrated_two_full_health_20261028.sample',
    validated_consecutive_recovery_snapshots=2,duplicate_transport_confirmation_removed=True,
    shutdown_read_attempts_per_snapshot=1,shutdown_read_budget_ms=200,
    shutdown_snapshot_uses_motion_retry_allowance=False,
    goal_writes_after_shutdown_prohibited=True)

class DirectPoseBus(pose.PoseBus):
    def __init__(self,report,cancel):
        super().__init__(report,cancel)
        self._last_health_finished=None
        self._shutdown_read_only=False
        self._read_recovery_times=[]
        report['direct_transport_policy']=dict(POLICY)

    def snapshot(self):
        if getattr(self,'_shutdown_read_only',False):return self._shutdown_snapshot()
        last=getattr(self,'_last_health_finished',None)
        if self.controlled and last is not None:
            until=last+HEALTH_READ_GAP_SECONDS
            while clock()<until:
                self.cancel_callback()
                if clock()>=getattr(self,'_horizontal_sample_deadline',float('inf')):
                    raise RuntimeError('Health read spacing exceeded existing sample deadline; no target advance')
                sleep(min(.002,max(0,until-clock())))
        try:
            if self.controlled and getattr(self,'_horizontal_sample_deadline',None) is not None:
                return self._motion_snapshot()
            return super().snapshot()
        finally:self._last_health_finished=clock()

    def _motion_snapshot(self):
        from grip_health_read_20261118 import block_once
        from combined_quiet_reserve_20261029 import receive
        def operation():
            state,comm,error,details=block_once(self,64,83,pose.FIELDS)
            if not comm and not error:
                if state is None or any(name not in state or np.asarray(state[name]).shape!=(16,) for name,_,_ in pose.FIELDS):
                    raise RuntimeError('Incomplete motion full16 state; no target advance')
            return state,comm,error,details
        # Only used inside the existing outer sample deadline. After a bad
        # packet, return the first complete snapshot for health/goal/held-joint
        # validation. The outer sampler then reads and validates the second;
        # neither snapshot advances a goal until that confirmation succeeds.
        return receive(self,operation,'Direct full16 health read',
            dict(kind='sync_read',address=64,length=83),now=clock,sleep=sleep)

    def _shutdown_snapshot(self):
        from grip_health_read_20261118 import block_once
        began=clock();previous=getattr(self,'_horizontal_sample_deadline',None)
        self._horizontal_sample_deadline=min(began+.2,previous if previous is not None else float('inf'))
        event=dict(full16_required=True,attempts=1,motion_retry_allowance_used=False,verified=False)
        self.report.setdefault('shutdown_verification_reads',[]).append(event)
        try:
            state,comm,error,details=block_once(self,64,83,pose.FIELDS)
            event.update(communication=int(comm),device_error=int(error),details=details)
            if comm or error or state is None:
                raise RuntimeError(f'Shutdown full16 read failed: comm={comm}, error={error}')
            if clock()>=self._horizontal_sample_deadline:
                raise RuntimeError('Shutdown full16 read exceeded200ms')
            if any(name not in state or np.asarray(state[name]).shape!=(16,) for name,_,_ in pose.FIELDS):
                raise RuntimeError('Incomplete shutdown full16 state')
            event.update(verified=True,all_torque_off=bool(not np.any(state['torque'])))
            return state
        except Exception as exc:
            event['error']=str(exc);raise
        finally:
            event['elapsed_ms']=(clock()-began)*1000
            if previous is None:del self._horizontal_sample_deadline
            else:self._horizontal_sample_deadline=previous

    def off(self):
        self._shutdown_read_only=True
        # Reuse the original broadcast OFF, individual fallback and full16 OFF
        # check. Only its snapshot transport bypasses motion retry accounting.
        return super().off()

    def write(self,address,size,values):
        if getattr(self,'_shutdown_read_only',False):
            if address==116 or (address==64 and np.any(values)):
                raise RuntimeError('Goal/enable write prohibited after shutdown began')
        return super().write(address,size,values)
