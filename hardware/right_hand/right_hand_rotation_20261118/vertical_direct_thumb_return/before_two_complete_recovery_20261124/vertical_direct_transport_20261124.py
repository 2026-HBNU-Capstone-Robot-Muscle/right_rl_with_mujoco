"""Direct-cycle transport spacing and bounded shutdown-only verification.

Imports open no devices. Motion uses the original full-health reader and its
200ms / three-retries-per-10s rules. After OFF begins, only fresh shutdown
verification reads are independent of the exhausted motion retry allowance.
"""
from time import perf_counter as clock, sleep
import numpy as np
import leap_measured_grip_hold_20261117 as pose

HEALTH_READ_GAP_SECONDS=.010
POLICY=dict(version='direct_health_gap_shutdown_verify_v1_20261124',
    minimum_complete_health_read_gap_ms=10,full16_required=True,
    motion_recovery_policy_unchanged=True,motion_sample_budget_ms=200,
    shutdown_read_attempts_per_snapshot=1,shutdown_read_budget_ms=200,
    shutdown_snapshot_uses_motion_retry_allowance=False,
    goal_writes_after_shutdown_prohibited=True)

class DirectPoseBus(pose.PoseBus):
    def __init__(self,report,cancel):
        super().__init__(report,cancel)
        self._last_health_finished=None
        self._shutdown_read_only=False
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
        try:return super().snapshot()
        finally:self._last_health_finished=clock()

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
