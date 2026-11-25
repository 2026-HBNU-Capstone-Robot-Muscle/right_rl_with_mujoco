"""Bounded READ-only OFF settling confirmation; no target or setting writes."""
import time
import numpy as np

POLICY=dict(version='off_preload_three_fresh_span2_v1_20261027',budget_seconds=.25,
            fresh_reads_required=3,position_span_ticks=2,goal_span_ticks=2,
            original_position_window_ticks=4,original_goal_window_ticks=4,
            maximum_reads=7,extra_goal_writes=0,extra_setting_writes=0,
            torque_enable_checks_unchanged=True)

def confirm_off_preload(bus,original,report,*,cancel,sleep=time.sleep,now=time.monotonic):
    from leap_roots_empty import check_off
    began=now();recent=[];log=dict(POLICY,reads=0,accepted=False)
    report['off_preload_confirmation']=log
    for _ in range(POLICY['maximum_reads']):
        cancel()
        if now()-began>=POLICY['budget_seconds']:break
        state=bus.snapshot();log['reads']+=1
        report.setdefault('off_preload_snapshots',[]).append({k:np.asarray(v).tolist() for k,v in state.items()})
        # These existing checks retain OFF/error/watchdog, voltage, temperature,
        # configured gains/current/profiles and the four-tick preload bounds.
        check_off(state,original,configured=True)
        bus.plan_module.validate_start(state['position'])
        elapsed=now()-began;log['elapsed_seconds']=elapsed
        if elapsed>POLICY['budget_seconds']:break
        recent.append(state);recent=recent[-POLICY['fresh_reads_required']:]
        if len(recent)==POLICY['fresh_reads_required']:
            positions=np.ptp(np.array([s['position'] for s in recent]),axis=0)
            goals=np.ptp(np.array([s['goal'] for s in recent]),axis=0)
            log['observed_position_span_ticks']=positions.tolist();log['observed_goal_span_ticks']=goals.tolist()
            if np.all(positions<=POLICY['position_span_ticks']) and np.all(goals<=POLICY['goal_span_ticks']):
                log['accepted']=True
                return state
        sleep(.02)
    log['elapsed_seconds']=now()-began
    raise RuntimeError('OFF preload did not settle across3 fresh reads within250ms; no enable')
