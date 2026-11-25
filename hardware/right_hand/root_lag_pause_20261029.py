"""Hold the last goal while the middle root catches up. Never writes a goal."""
import time
POLICY=dict(version='middle_root_preemptive_wait_v1_20261029',motor_id=12,phase='stage_7_horizontal_yaw',
    enter_error_rad=.12,resume_error_rad=.10,consecutive_good_samples=2,
    maximum_single_wait_s=1.2,maximum_total_wait_s=20.,period_s=.075,
    fresh_read_reservation_s=.2,original_health_checks_and_progress_guard=True,
    original_tracking_stop_rad=.18,goal_writes_during_wait=0,
    wait_counts_toward_original_motion_timeout=True)
class RootLagPause:
    def __init__(self,*,now=time.perf_counter):self.now=now;self.total=0.
    def wait_if_needed(self,phase,state,errors,cycle_start,*,observe,command,report,sleep,cycle_now):
        if phase!=POLICY['phase'] or float(errors[11])<POLICY['enter_error_rad']:return state,errors,cycle_start
        began=self.now();budget=min(POLICY['maximum_single_wait_s'],POLICY['maximum_total_wait_s']-self.total)
        event=dict(phase=phase,command=list(map(int,command)),start_error_rad=float(errors[11]),start_current_raw=int(state['current'][11]) if 'current' in state else None,
            goal_writes=0,status='waiting',sample_indices=[],entered_sample_index=len(report.get('samples',[]))-1)
        report.setdefault('root_lag_wait_events',[]).append(event);report['root_lag_pause_policy']=dict(POLICY)
        clean=0
        try:
            if budget<=POLICY['fresh_read_reservation_s']:raise RuntimeError('ID12 catch-up wait total budget exhausted; no target advance')
            print(f'ID12 지연 {float(errors[11]):.3f}rad: 현재 목표를 유지하며 따라오기를 기다립니다.',flush=True)
            while True:
                remaining=budget-(self.now()-began)
                if remaining<POLICY['fresh_read_reservation_s']+POLICY['period_s']:
                    raise RuntimeError('ID12 did not settle within bounded wait; no target advance')
                sleep(POLICY['period_s'])
                start=self.now();state,errors,_=observe()  # Original health, held and progress checks.
                if self.now()-began>=budget:raise RuntimeError('ID12 catch-up wait deadline reached; no target advance')
                event['sample_indices'].append(len(report.get('samples',[]))-1)
                event.update(last_error_rad=float(errors[11]),last_position_ticks=int(state['position'][11]))
                clean=clean+1 if float(errors[11])<=POLICY['resume_error_rad'] else 0
                if clean>=POLICY['consecutive_good_samples']:
                    event.update(status='resumed',consecutive_good_samples=clean)
                    return state,errors,cycle_now()
                # No catch-up scheduling. Every observation is at least75ms
                # after the previous completed observation; slow reads add time.
        except BaseException as exc:
            event.update(status='stopped',error=str(exc));raise
        finally:
            duration=self.now()-began;self.total+=duration
            event['elapsed_seconds']=duration;report['root_lag_wait_total_seconds']=self.total
