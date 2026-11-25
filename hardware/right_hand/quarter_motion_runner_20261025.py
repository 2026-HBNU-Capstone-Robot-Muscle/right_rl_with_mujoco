"""Dedicated quarter-path runner; old shared motion tools are untouched."""
import time
from collections import deque
import numpy as np
import sequence_flip_checked as checked
from leap_roots_empty import check_off
from goal_aware_progress_guard_20261025 import GoalAwareProgress,GoalAwareProgressFault,metadata as progress_metadata

def run(bus,initial,report,*,cancel=checked.check_cancel,sleep=time.sleep,now=time.monotonic,
        prepared_callback=None,phase_callback=None,release_callback=None):
    p=bus.plan_module
    checkpoint=getattr(p,'CAPTURE_BEFORE_PHASE',None)
    if checkpoint and (phase_callback is None or prepared_callback is None):
        raise RuntimeError('Loaded approach needs both PLACE and REMOVE callbacks; no writes')
    release_phase=getattr(p,'CONTACT_RELEASE_BEFORE_PHASE',None)
    if release_phase and release_callback is None:
        raise RuntimeError('Measured contact release needs an observation callback; no writes')
    original=p.validate_start(initial['position']);check_off(initial,original)
    bus.bind(original)
    report.update(initial={k:v.tolist() for k,v in initial.items()},source_sha256=p.SOURCE_SHA,
        source_path=str(p.SOURCE),start_difference_ticks=(original-p.source_position()).tolist(),
        settings_restored=False,all_torque_off_verified=False,thumb_max_goal_travel_ticks=p.MAX_THUMB_TRAVEL)
    report['cycle_pacing']=('processing_included' if hasattr(p,'SAMPLE_PERIOD') else 'fixed_sleep_after_processing')
    report['requested_sample_period_s']=getattr(p,'SAMPLE_PERIOD',None)
    cancel();check_off(bus.snapshot(),original)
    try:
        report['phase']='configure_off'
        for addr,key in ((80,'d_gain'),(82,'i_gain'),(84,'p_gain')):bus.write(addr,2,checked.GAINS[key])
        bus.write(102,2,checked.CAP)
        bus.write(108,4,np.zeros(16,dtype=int));bus.write(112,4,np.zeros(16,dtype=int))
        bus.write(116,4,original,verify=False)
        previous=None
        for _ in range(2):
            cancel();s=bus.snapshot()
            report.setdefault('off_preload_snapshots',[]).append({k:v.tolist() for k,v in s.items()})
            check_off(s,original,configured=True)
            if previous is not None and any(np.any(abs(s[k]-previous[k])>1) for k in ('position','goal')):
                raise RuntimeError('OFF preload unstable; no enable')
            previous=s;sleep(.02)
        bus.write(98,1,np.full(16,25));cancel();bus.write(64,1,np.ones(16,dtype=int))
        report['enabled_at']=now();report['phase']='enable_hold';previous=None
        for _ in range(2):
            cancel();s=bus.snapshot()
            report.setdefault('enable_snapshots',[]).append({k:v.tolist() for k,v in s.items()})
            checked.health(s,s['goal'])
            if any(np.any(abs(a)>4) for a in (s['position']-original,s['goal']-original,s['goal']-s['position'])):
                raise RuntimeError('Enable changed raw hold beyond four ticks; OFF')
            if previous is not None and any(np.any(abs(s[k]-previous[k])>1) for k in ('position','goal')):
                raise RuntimeError('Enable hold unstable; OFF')
            previous=s;sleep(.02)
        command=s['goal'].copy();report['last_issued_command']=command.tolist();bus.adopt(command)
        # Only an opted-in current plan may confirm an old packet. Enable and
        # preload remain strict; each actual accepted command stays unchanged.
        bus.goal_readback_confirmation=bool(getattr(p,'CONFIRM_PREVIOUS_GOAL_READBACK',False))
        if bus.goal_readback_confirmation:
            report['goal_readback_policy']=dict(version='exact_previous_packet_fresh_confirm_v1',
                confirm_seconds=.15,max_fresh_reads=3,exact_reads_required=2,
                maximum_previous_step_ticks=2,goal_write_retries=0)
        fixed=command.copy();rows=p.plan(command)
        report.update(accepted_enable_goal=command.tolist(),enable_goal_shift_ticks=(command-original).tolist(),
            planned_final_command=rows[-1][1].tolist(),final_command=command.tolist(),thumb_travel_ticks=(rows[-1][1][:4]-command[:4]).tolist())
        bus.write(116,4,command)

        held_guard=None
        if getattr(p,'HELD_JOINT_GUARD',False):
            from held_joint_guard import HeldJointGuard
            held_guard=HeldJointGuard(s['position'],command)
            report['held_joint_guard']=held_guard.VERSION

        progress_history={};progress_guard=GoalAwareProgress();report['progress_policy']=progress_metadata();paused_seconds=0.;checkpoint_done=False;release_done=False
        def observe(phase):
            cycle_start=now()
            cancel()
            timeout=getattr(p,'ON_TIMEOUT',35)
            if cycle_start-report['enabled_at']-paused_seconds>timeout:
                raise RuntimeError(f'Preparation exceeded {timeout}s; OFF')
            frozen=p.frozen_for_phase(phase) if hasattr(p,'frozen_for_phase') else p.FROZEN
            progress=(p.progress_for_phase(phase) if hasattr(p,'progress_for_phase')
                      else np.arange(4) if phase=='stage_1_thumb_prefix' else np.array([],dtype=int))
            def check_snapshot(state):
                if held_guard is not None:
                    from held_joint_guard import HeldJointFault
                    try:held_guard.check(state['position'],command,frozen,phase)
                    except HeldJointFault as exc:
                        report['fixed_hold_fault']=exc.details
                        raise
                elif np.any(abs(state['position'][frozen]-command[frozen])>20):
                    raise RuntimeError('A fixed finger joint moved over 20 ticks; no target advance')
                if len(progress):
                    try:progress_guard.check(phase,progress,command,state['position'],now())
                    except GoalAwareProgressFault as exc:
                        report['progress_fault']=exc.details
                        raise
            state,errors=checked.sample(bus,command,report,phase,snapshot_check=check_snapshot)
            return state,errors,cycle_start

        def finish_cycle(cycle_start):
            period=getattr(p,'SAMPLE_PERIOD',None)
            if period is None:sleep(.05)
            else:
                # Schedule each cycle independently. No catch-up bursts and no
                # skipped targets; each next goal still requires a fresh read.
                remaining=max(0.,period-(now()-cycle_start))
                if remaining:sleep(remaining)

        for i,(phase,target) in enumerate(rows,1):
            if phase==release_phase and not release_done:
                report['phase']='release_checkpoint'
                state,_,_=observe('release_checkpoint')
                contact_cases=[s for s in bus.proof['static_screens'] if s['initial_self_contacts_mm']]
                threshold=(min(s['contact_clear_pip_goal_ticks'] for s in contact_cases)-p.CONTACT_RELEASE_MARGIN_TICKS
                           if contact_cases else None)
                report['contact_release_threshold_ticks']=threshold
                report['contact_release_actual_ticks']=int(state['position'][14])
                if threshold is not None and state['position'][14]>threshold:
                    raise RuntimeError(f'ID15 has not reached the certified release: actual={state["position"][14]}, need<={threshold}; no thumb/opening advance')
                began=now();release_callback(bus,command.copy(),report,held_guard)
                elapsed=now()-began
                if elapsed>100 or not report.get('contact_release_operator_confirmed'):
                    raise RuntimeError('Contact release was not confirmed; no thumb/opening advance')
                paused_seconds+=elapsed;report['contact_release_pause_seconds']=elapsed
                report['operator_pause_seconds_total']=paused_seconds;release_done=True
            if phase==checkpoint and not checkpoint_done:
                # The queue still contains the entire loaded segment. Waiting
                # holds the last EMPTY goal; it cannot consume or replace a
                # goal. The protocol requires fresh SPACE and ongoing health.
                if i==1 or held_guard is None:raise RuntimeError('Invalid placement checkpoint; OFF')
                for _ in range(getattr(p,'CHECKPOINT_SETTLE_SAMPLES',1)):
                    state,_,cycle_start=observe('placement_checkpoint');finish_cycle(cycle_start)
                if np.any(abs(state['position']-command)>getattr(p,'CHECKPOINT_MAX_ERROR_TICKS',4096)):
                    raise RuntimeError('Open preparation has not settled within its placement tolerance; no loaded target advance')
                began=now()
                phase_callback(bus,command.copy(),report,held_guard)
                elapsed=now()-began
                if elapsed>getattr(p,'PLACEMENT_PAUSE_LIMIT',100):
                    raise RuntimeError('Placement checkpoint exceeded its time limit; OFF')
                if not report.get('cube_placement_confirmed'):
                    raise RuntimeError('No explicit cube placement confirmation; no loaded target advance')
                paused_seconds+=elapsed;report['placement_pause_seconds']=elapsed
                report['operator_pause_seconds_total']=paused_seconds
                checkpoint_done=True
            report['phase']=phase
            state,errors,cycle_start=observe(phase)
            bus.write(116,4,target,verify=False)
            if held_guard is not None:held_guard.advance(state['position'],command,target)
            command=target.copy();report['last_issued_command']=command.tolist();report['last_issued_phase']=phase
            if i%30==0:print(f'{phase}: {i}/{len(rows)}; tracking {max(errors):.3f}rad',flush=True)
            finish_cycle(cycle_start)
        if checkpoint and not checkpoint_done:raise RuntimeError('Placement checkpoint missing from path; OFF')
        report['phase']='stage_2_fixed_hold'
        for _ in range(20):
            state,errors,cycle_start=observe(report['phase']);finish_cycle(cycle_start)
        report['final_on_state']={k:v.tolist() for k,v in state.items()}
        if prepared_callback is not None:prepared_callback(bus,command,report)
        report['status']=getattr(p,'STATUS','empty_thumb_prefix_complete')
    finally:
        if 'command' in locals():
            report['last_issued_command']=command.tolist();report['final_command']=command.tolist()
        bus.off();report['all_torque_off_verified']=True
        bus.write(98,1,np.zeros(16,dtype=int))
        for addr,size,key in ((102,2,'current_cap'),(108,4,'acceleration'),(112,4,'velocity'),
            (98,1,'watchdog'),(80,2,'d_gain'),(82,2,'i_gain'),(84,2,'p_gain')):
            bus.write(addr,size,initial[key])
        report['settings_restored']=True
