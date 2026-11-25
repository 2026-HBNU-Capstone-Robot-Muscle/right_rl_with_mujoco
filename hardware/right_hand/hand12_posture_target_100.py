"""Empty-hand controlled-posture ID12 test: other joints first, then ID12.
NOT policy control. Original policy and preparation limits are unchanged.
"""
import datetime
import json
import time
import numpy as np
import hand_start_pose as base
from hand_pip_inward_recovery import recovery_envelope
from hand12_start_target_100 import make_target as id12_target

TICK = base.TICK
CAPS, STOPS = base.current_profile(100)
CAPS[11], STOPS[11] = 100, 110
CAPS.flags.writeable = STOPS.flags.writeable = False


class TrackingPace:
    """Pause target advance before a tracking fault; never re-enable or add force."""
    PAUSE_RAD = .030
    # A loaded servo can settle with a small residual error. Resume within
    # 0.020rad (below the 0.030rad pause band and existing final pose tolerance),
    # leaving >3 command increments of headroom. Do not demand near-zero error.
    RESUME_RAD = .020
    MAX_WAIT_S = 2.0

    def __init__(self, report, phase, *, settled_error=RESUME_RAD):
        if not np.isfinite(settled_error) or not 0 < settled_error <= .035:
            raise ValueError('Tracking pace must remain within the existing settled-pose tolerance')
        self.events = report.setdefault('tracking_wait_events', [])
        self.phase = phase
        self.waiting_since = None
        # A pose accepted at the end of this phase must not be rejected merely
        # for the same residual error during motion. Four ticks separate bands
        # of hysteresis; adaptive steps slow before this band. Hard checks remain separate.
        self.resume_rad = settled_error
        self.pause_rad = max(self.PAUSE_RAD, settled_error + 4*TICK)
        report.setdefault('tracking_pace_by_phase', {})[phase] = dict(
            pause_rad=self.pause_rad, resume_rad=self.resume_rad, max_wait_s=self.MAX_WAIT_S)

    def ready(self, error, stamp, motor):
        if self.waiting_since is not None:
            # A late sample must not reset the bounded wait, even if it caught up.
            if stamp-self.waiting_since >= self.MAX_WAIT_S:
                raise RuntimeError(f'ID {motor} tracking did not recover within 2s; no re-enable or current increase')
            if error <= self.resume_rad:
                self.events.append(dict(phase=self.phase, event='resume', t=stamp,
                                        waited_s=stamp-self.waiting_since))
                self.waiting_since = None
                return True
            return False
        if error >= self.pause_rad:
            self.waiting_since = stamp
            self.events.append(dict(phase=self.phase, event='wait', t=stamp,
                                    motor_id=motor, error_rad=float(error)))
            return False
        return True


class AdaptiveSteps:
    """One-way per-joint slowdown before the existing tracking pause band."""
    def __init__(self, report, phase, *, caps=CAPS, max_step_ticks=4):
        self.steps=np.full(16,max_step_ticks,dtype=int)
        self.caps=np.array(caps,copy=True)
        self.report,self.phase=report,phase

    def update(self, error, current, stamp):
        current=np.asarray(current)
        current=np.abs(np.where(current>=32768,current-65536,current))
        loaded=(error>=.010)&(current>=.9*self.caps)
        next_steps=self.steps.copy()
        next_steps[(error>=.010)&(next_steps>4)]=4
        next_steps[(error>=.020)|loaded]=2
        next_steps=np.minimum(self.steps,next_steps)
        slow=next_steps<self.steps
        for i in np.flatnonzero(slow):
            reason='current_near_cap' if loaded[i] else 'tracking_error'
            self.report.setdefault('adaptive_step_events',[]).append(dict(
                phase=self.phase,t=float(stamp),motor_id=int(i)+1,
                step_ticks_before=int(self.steps[i]),step_ticks_after=int(next_steps[i]),reason=reason,
                error_rad=float(error[i]),current_ma=int(current[i])))
            print(f'{self.phase}: ID{i+1} slow {self.steps[i]} -> {next_steps[i]} ticks ({reason}); continuing',flush=True)
        self.steps=next_steps
        self.report.setdefault('final_step_ticks_by_phase',{})[self.phase]=self.steps.tolist()
        return self.steps.copy()


def plan(raw):
    recovery_envelope(raw)
    id12_target(raw)  # Same ID12 absolute limits, fixed target and 0.45rad budget.
    raw = np.asarray(raw, dtype=int)
    staged = raw.copy()
    moves = []
    for i in (6, 10):
        if raw[i]*TICK-base.OFFSET < -.32:
            target = min(int(np.ceil((base.OFFSET-.30)/TICK)),
                         int(raw[i])+int(np.floor(.15/TICK))-4)
            staged[i] = target
            moves.append((i, target))
    # No movement unless one bounded recovery step suffices for preparation.
    goal = base.target_plan(staged)
    if np.max(np.abs(goal-raw))*TICK > 1.40:
        raise ValueError('Total staged travel exceeds 1.40rad; no motion')
    return moves, staged, goal


class Bus(base.SyncBus):
    planner = staticmethod(plan)
    current_caps = CAPS

    def preflight(self):
        for motor in base.IDS:
            model,comm,error = self.packet.ping(self.port,motor)
            self.report.setdefault('ping_events', []).append(dict(
                motor_id=motor, model_number=int(model), communication=int(comm), device_error=int(error)))
            if comm != self.sdk.COMM_SUCCESS:
                detail = 'no response (timeout)' if comm == -3001 else 'communication failed'
                raise RuntimeError(f'COM7 at 3000000 baud: ID {motor} {detail}, code={comm}. '
                                   'Check motor power and the adapter-to-hand cable; no motion started.')
            if error:
                raise RuntimeError(f'ID {motor}: device error {error}, model={model}; no motion started')
            if model != 1240:
                raise RuntimeError(f'ID {motor}: model={model}, expected XC330-M288 (1240); no motion started')
            for addr,value in ((64,0),(70,0),(10,0),(11,5),(98,0)):
                if self.read(motor,addr) != value:
                    raise RuntimeError(f'ID {motor}: unexpected register {addr}')
            if self.read(motor,6)<38 or self.read(motor,20,4)!=0:
                raise RuntimeError(f'ID {motor}: firmware/offset mismatch')
            if self.read(motor,38,2)<self.current_caps[motor-1]:
                raise RuntimeError(f'ID {motor}: current limit below trial cap')
        state = self.snapshot()
        if np.any(state['torque']) or np.any(state['error']) or np.any(state['watchdog']):
            raise RuntimeError('All motors must begin OFF without error/watchdog')
        if np.any(state['temperature']>45) or np.any(state['voltage']<35) or np.any(state['voltage']>65):
            raise RuntimeError('Temperature/voltage outside trial bounds')
        self.report['preflight_state'] = pack(state)
        self.planner(state['position'])
        return state


def pack(state):
    return {k:v.tolist() for k,v in state.items()}


def run_trial(bus, initial, report, cancel=base.check_cancel, now=time.monotonic, sleep=time.sleep, *, held_check=None, planner=plan, full_start=False, start_step_ticks=2, preload_check=None, active_check=None, current_caps=CAPS, profile_velocity=5, measured_current_stops=STOPS, total_on_seconds=60, preparation_limit_seconds=60, phase_timeouts=True, recovery_step_ticks=2, adaptive_factory=AdaptiveSteps, position_gains=None, policy_step_ticks=8, policy_radius_ticks=32, policy_joint_limits=False):
    if policy_radius_ticks not in (32,65): raise ValueError('Invalid bounded policy radius')
    if policy_joint_limits and (not full_start or active_check is None or held_check is not None or
                               not report.get('loaded_policy_mode') or not report.get('sac_deployment_mode')):
        raise ValueError('Joint-limit control requires explicit SAC deployment mode')
    if policy_radius_ticks!=32 and (not full_start or active_check is None or held_check is not None or not report.get('loaded_policy_mode')):
        raise ValueError('Expanded radius requires the explicit loaded-policy trial')
    if total_on_seconds not in (None,60,90): raise ValueError('Unsupported total ON limit')
    if (total_on_seconds is None or not phase_timeouts) and not full_start:
        raise ValueError('Stage timing requires full preparation')
    if total_on_seconds==90 and (not full_start or held_check is None or active_check is not None):
        raise ValueError('90s limit requires full preparation and fixed hold only')
    caps=np.array(current_caps,copy=True)
    stops=np.array(measured_current_stops,copy=True)
    if (stops.shape!=(16,) or not np.isfinite(stops).all() or
            np.any(stops!=np.rint(stops)) or np.any(stops<=0)):
        raise ValueError('Measured-current stops must be 16 positive integers')
    stops=stops.astype(int)
    if (caps.shape!=(16,) or not np.isfinite(caps).all() or np.any(caps!=np.rint(caps))
            or np.any(caps<=0) or np.any(caps>=stops)):
        raise ValueError('Current caps must be positive integers below measured-current stops')
    caps=caps.astype(int)
    if start_step_ticks not in (2,4,8,16) or recovery_step_ticks not in (2,8) or profile_velocity not in (5,20,60):
        raise ValueError('Invalid tested step/profile setting')
    gains = {} if position_gains is None else {k:np.asarray(v).copy() for k,v in position_gains.items()}
    if gains and set(gains) != {'p_gain','i_gain','d_gain'}:
        raise ValueError('Complete PID gains required')
    for name, values in gains.items():
        for v in (values,np.asarray(initial.get(name))):
            if v.shape!=(16,) or not np.isfinite(v).all() or np.any(v!=np.rint(v)) or np.any(v<0) or np.any(v>16383):
                raise ValueError('Invalid/missing PID gain state')
    original = initial['position'].copy()
    moves, staged, goal = planner(original)
    report['preparation_mode'] = 'full_start_pose' if full_start else 'isolated_id12_posture_test'
    report['profile_velocity_raw']=profile_velocity
    report.update(initial_state=pack(initial), recovery_moves=[list(x) for x in moves],
                  goal_hw_raw=goal.tolist(), samples=[], current_ma_by_hw_id=caps.tolist(),
                  measured_current_stop_ma_by_hw_id=stops.tolist(),
                  total_on_limit_s=total_on_seconds, preparation_limit_s=preparation_limit_seconds,
                  position_gains_by_hw_id={k:v.tolist() for k,v in gains.items()})
    touched = False
    began = None

    def check_time():
        elapsed=now()-began
        if total_on_seconds is not None and elapsed>total_on_seconds:
            raise RuntimeError(f'{total_on_seconds}-second total torque-on timeout')
        if preparation_limit_seconds is not None and report.get('phase') not in ('fixed_start_pose_check','bounded_policy_motion') and elapsed>preparation_limit_seconds:
            raise RuntimeError(f'Preparation exceeded {preparation_limit_seconds}-second limit')

    def check_gains(state):
        for name, values in gains.items():
            if name not in state or not np.array_equal(state[name],values):
                raise RuntimeError(f'Position gain changed: {name}')

    def sample(command, path_start, path_goal, threshold=.10):
        sample_began = now()
        cancel()
        cancel_ended = now()
        check_time()
        read_began = now()
        state = bus.snapshot()
        read_ended = now()
        err = np.abs(state['position']-command)*TICK
        worst = int(np.argmax(err))
        elapsed = read_ended-began
        previous_sample = report['samples'][-1]['t'] if report['samples'] else None
        report['samples'].append(dict(t=elapsed, phase=report['phase'],
            cancel_duration_ms=(cancel_ended-sample_began)*1000,
            snapshot_duration_ms=(read_ended-read_began)*1000,
            sample_interval_ms=None if previous_sample is None else (elapsed-previous_sample)*1000,
            command_hw_raw=command.tolist(), state_all_ids=pack(state),
            worst_tracking_id=worst+1, max_tracking_error_rad=float(err[worst])))
        if now()-read_began > .2:
            raise RuntimeError('Motor snapshot exceeded 200ms; no target advance')
        check_time()
        validate_motion(state,path_start,path_goal,command,expected_current_ma=caps,expected_profile_velocity=profile_velocity,
                        expected_current_stops=stops)
        check_gains(state)
        if np.max(err)>threshold:
            raise RuntimeError(f'ID {worst+1} phase tracking error {err[worst]:.3f}rad')
        return state

    def ramp(command, target, *, phase, timeout, threshold, settle, final_error, moving_index=None, hold_index=None, step_ticks=2):
        report['phase'] = phase
        report.setdefault('command_step_ticks_by_phase', {})[phase] = step_ticks
        start = command.copy()
        phase_began = previous = last_print = now()
        reached = None
        pace = TrackingPace(report, phase, settled_error=final_error)
        adaptive=adaptive_factory(report,phase,caps=caps,max_step_ticks=step_ticks) if full_start and step_ticks>=4 else None
        best_remaining=np.abs(target-start)
        last_progress=np.full(16,phase_began)
        while True:
            state = sample(command,start,target,threshold)
            stamp = now()
            if phase_timeouts and stamp-phase_began>timeout:
                raise RuntimeError(f'{phase}: phase timeout')
            if not phase_timeouts:
                remaining=np.abs(target-state['position'])
                progressed=remaining<=best_remaining-2
                last_progress[progressed]=stamp
                best_remaining[progressed]=remaining[progressed]
                stalled=(remaining*TICK>final_error)&(stamp-last_progress>5.)
                if np.any(stalled):
                    raise RuntimeError(f'ID {int(np.flatnonzero(stalled)[0])+1}: no preparation progress for 5s')
            if hold_index is not None and abs(int(state['position'][hold_index])-int(start[hold_index]))*TICK>.035:
                raise RuntimeError(f'ID {hold_index+1} fixed hold drift >0.035rad; no retry')
            if moving_index is not None:
                hold_errors = np.delete(np.abs(state['position']-start)*TICK,moving_index)
                if np.max(hold_errors)>.035:
                    raise RuntimeError('A held joint drifted >0.035rad during recovery')
            error = np.abs(state['position']-command)*TICK
            steps=adaptive.update(error,state['current'],stamp) if adaptive is not None else step_ticks
            worst = int(np.argmax(error))
            advance = pace.ready(float(error[worst]), stamp, worst+1)
            if stamp-last_print>=1:
                s=report['samples'][-1]
                print(f'{phase}: ID{s["worst_tracking_id"]} largest error {s["max_tracking_error_rad"]:.3f}rad',flush=True)
                last_print=stamp
            if np.array_equal(command,target):
                reached = stamp if reached is None else reached
                if stamp-reached>=settle:
                    if np.max(np.abs(state['position']-target))*TICK>final_error:
                        raise RuntimeError(f'{phase}: target not reached; no retry')
                    return command,state
            if advance and stamp-previous>=.05 and not np.array_equal(command,target):
                # Logging/UI can block after the read. Never resume with a stale target advance.
                age=now()-(began+report['samples'][-1]['t'])
                if age>.2:
                    report['stale_preparation_advance_ms']=age*1000
                    raise RuntimeError(f'Preparation state became stale before goal write ({age*1000:.0f}ms); no target advance')
                command=command+np.clip(target-command,-steps,steps)
                bus.write(116,4,command,verify=False)
                previous=stamp
            sleep(.01)

    try:
        cancel()
        if np.any(initial['torque']):
            raise RuntimeError('All motors must begin torque OFF')
        touched=True
        report['phase']='configure_preload_OFF'
        for addr,name in ((80,'d_gain'),(82,'i_gain'),(84,'p_gain')):
            if gains: bus.write(addr,2,gains[name])
        for addr,size,values in ((102,2,caps),(108,4,np.full(16,5)),(112,4,np.full(16,profile_velocity))):
            bus.write(addr,size,values)
        bus.write(116,4,original,verify=False)
        report['preload_samples']=[]
        if preload_check is not None:
            preloaded=preload_check(bus,original,report,cancel=cancel,now=now,sleep=sleep)
        else:
            for _ in range(2):
                state=bus.snapshot()
                report['preload_samples'].append(pack(state))
                preloaded=base.validate_preload(state,original,expected_current_ma=caps,expected_profile_velocity=profile_velocity)
                sleep(.02)
        if gains: check_gains(bus.snapshot())
        bus.write(98,1,np.full(16,25))
        cancel()
        report['phase']='enable_hold_all'
        began=now()
        bus.write(64,1,np.ones(16,dtype=int))
        first=None
        report['enable_samples']=[]
        for _ in range(2):
            cancel()
            state=bus.snapshot()
            report['enable_samples'].append(pack(state))
            if now()-began>.5:
                raise RuntimeError('Enable transition timeout')
            observed=state['goal']
            validate_motion(state,original,original,observed,expected_current_ma=caps,expected_profile_velocity=profile_velocity,
                            expected_current_stops=stops)
            check_gains(state)
            for value in (observed,state['position']):
                if np.any(np.abs(value-original)>4) or np.any(np.abs(value-preloaded)>4):
                    raise RuntimeError('Enable shift exceeds 4 ticks')
            if np.any(np.abs(state['position']-observed)>1):
                raise RuntimeError('Enable hold differs from actual position')
            stable_ticks=4 if full_start else 1
            goal_changed=(np.any(np.abs(observed-first['goal'])>4) if full_start else
                          not np.array_equal(observed,first['goal'])) if first is not None else False
            if first is not None and (goal_changed or
                    np.any(np.abs(state['position']-first['position'])>stable_ticks)):
                raise RuntimeError('Unstable enable hold')
            first=state
            sleep(.02)
        command=first['goal'].copy()
        # Keep original targets; budget sequential segments from the accepted
        # hold and then the previous segment's goal, including repeated IDs.
        recovery_cursor=command.copy()
        for i,target in moves:
            if not 0<(target-recovery_cursor[i])*TICK<=.15:
                raise RuntimeError('Recovery travel exceeds bounded enable margin')
            recovery_cursor[i]=target
        bus.write(116,4,command)
        for i,target in moves:
            next_target=command.copy(); next_target[i]=target
            command,state=ramp(command,next_target,phase=f'recover_ID{i+1}',timeout=12,
                threshold=.075,settle=1,final_error=.02,moving_index=i,step_ticks=recovery_step_ticks)
        # Fresh measured angles, not just commands, must pass the ORIGINAL planner.
        state=sample(command,command,command,.035)
        if not np.array_equal(base.target_plan(state['position']),goal):
            raise RuntimeError('Measured recovery pose does not match start target')
        if not np.array_equal(base.target_plan(command),goal):
            raise RuntimeError('Commanded recovery pose does not match start target')
        report['recovered_state_before_start']=pack(state)
        if full_start:
            # The original full-hand planner above enforces the 1.25rad
            # preparation budget on both measured and commanded recovery poses.
            command,state=ramp(command,goal,phase='full_start_pose',timeout=45,
                threshold=.10,settle=2,final_error=.035,step_ticks=start_step_ticks)
        else:
            # Preserve the separate ID12 diagnostic's narrow 0.45rad contract.
            others_goal=goal.copy()
            others_goal[11]=command[11]
            command,state=ramp(command,others_goal,phase='prepare_other_15',timeout=45,
                threshold=.10,settle=1,final_error=.035,hold_index=11)
            report['before_id12_state']=pack(state)
            report['id12_hold_raw']=int(command[11])
            if not 0<(goal[11]-command[11])*TICK<=.45:
                raise RuntimeError('ID12 command travel exceeds 0.45rad')
            if not 0<(goal[11]-state['position'][11])*TICK<=.45:
                raise RuntimeError('ID12 measured travel exceeds 0.45rad')
            q=base.hw_to_sim(state['position']*TICK)
            if np.any(q<base.limits.SAFE_LOWERS) or np.any(q>base.limits.SAFE_UPPERS):
                raise RuntimeError('Prepared measured posture outside policy range')
            command,state=ramp(command,goal,phase='ID12_in_fixed_posture',timeout=25,
                threshold=.075,settle=2,final_error=.02,moving_index=11)
        report['final_sim_angles_before_off']=base.hw_to_sim(state['position']*TICK).tolist()
        if active_check is not None:
            if not full_start or held_check is not None:
                raise ValueError('Active trial requires full start and no fixed hold callback')
            from bounded_policy import Control
            report['phase']='bounded_policy_motion'
            control=Control(goal,
                read=lambda command,low,high:sample(command,low,high,.10),
                send=lambda target:bus.write(116,4,target),report=report,now=now,step_ticks=policy_step_ticks,
                radius_ticks=policy_radius_ticks,joint_limits=policy_joint_limits)
            active_check(control)
            control.read()
        if held_check is not None:
            report['phase']='fixed_start_pose_check'
            # Continue without OFF/recovery repetitions. Only checked reads are
            # exposed to shadow inference; the callback cannot send policy goals.
            held_check(lambda: sample(goal,goal,goal,.035))
            sample(goal,goal,goal,.035)
        report['status']='start_pose_reached_before_torque_off' if full_start else 'posture_controlled_id12_target_reached_before_off'
    except BaseException:
        report['failed_phase']=report.get('phase')
        raise
    finally:
        if touched:
            report['all_torque_off_verified']=False
            bus.off()
            report['all_torque_off_verified']=True
            bus.write(98,1,np.zeros(16,dtype=int))
            for addr,size,name in ((102,2,'current_cap'),(108,4,'acceleration'),(112,4,'velocity'),(98,1,'watchdog')):
                bus.write(addr,size,initial[name])
            for addr,name in ((80,'d_gain'),(82,'i_gain'),(84,'p_gain')):
                if gains: bus.write(addr,2,initial[name])
            report['settings_restored']=True


def validate_motion(state, start, goal, command, *, expected_current_ma=CAPS, expected_profile_velocity=5, expected_current_stops=STOPS):
    caps, stops = expected_current_ma, expected_current_stops
    if np.any(state['torque'] != 1) or np.any(state['error']) or np.any(state['watchdog'] != 25):
        details=[]
        expired=np.flatnonzero(np.isin(state['watchdog'],[-1,255]))+1
        if len(expired):
            details.append(f'Bus watchdog expired on IDs {expired.tolist()} (communication limit 500ms)')
        for key,expected in [('torque',1),('error',0),('watchdog',25)]:
            bad=np.flatnonzero(state[key]!=expected)
            if len(bad): details.append(key+' '+str({int(i)+1:int(state[key][i]) for i in bad}))
        raise RuntimeError('Torque/error/watchdog state changed: '+'; '.join(details))
    if np.any(state['goal'] != command):
        i = int(np.flatnonzero(state['goal'] != command)[0])
        raise RuntimeError(f'ID {i+1} goal changed: wrote={command[i]}, read={state["goal"][i]}')
    if np.any(state['current_cap'] != caps):
        raise RuntimeError('Current cap changed')
    if np.any(state['acceleration'] != 5) or np.any(state['velocity'] != expected_profile_velocity):
        raise RuntimeError('Profile limit changed')
    if np.any(state['temperature'] > 45) or np.any(state['voltage'] < 35) or np.any(state['voltage'] > 65):
        raise RuntimeError('Temperature/voltage outside trial bounds')
    current = np.where(state['current'] >= 32768,state['current']-65536,state['current'])
    if np.any(np.abs(current) > stops):
        i = int(np.flatnonzero(np.abs(current) > stops)[0])
        raise RuntimeError(f'ID {i+1} measured input current {current[i]}mA above trial threshold {stops[i]}mA')
    position = state['position']
    if np.any(position < np.minimum(start,goal)-20) or np.any(position > np.maximum(start,goal)+20):
        raise RuntimeError('Position outside planned path envelope')
    errors = np.abs(position-command)*TICK
    if np.max(errors) > base.MAX_TRACKING_ERROR:
        i = int(np.argmax(errors))
        raise RuntimeError(f'ID {i+1} tracking error {errors[i]:.3f}rad: '
                           f'goal={command[i]}, actual={position[i]}; no stronger retry')


def summarize(report):
    rows=report.get('samples',[])
    summary={k:report.get(k) for k in ('status','error','failed_phase','all_torque_off_verified','settings_restored')}
    summary['tracking_waits'] = sum(x['event']=='wait' for x in report.get('tracking_wait_events', []))
    summary['tracking_resumes'] = sum(x['event']=='resume' for x in report.get('tracking_wait_events', []))
    if rows:
        states=[x['state_all_ids'] for x in rows]
        currents=[int(x['current'][11]) for x in states]
        currents=[x-65536 if x>=32768 else x for x in currents]
        summary.update(last_phase=rows[-1]['phase'],id12_actual_raw=states[-1]['position'][11],
            id12_goal_raw=rows[-1]['command_hw_raw'][11],peak_abs_current_id12_ma=max(abs(x) for x in currents),
            min_voltage_id12_v=min(x['voltage'][11] for x in states)/10,
            max_temperature_id12_c=max(x['temperature'][11] for x in states))
    return summary


def main():
    report=dict(timestamp=datetime.datetime.now().astimezone().isoformat(),status='not_started',policy_used=False)
    bus=None; opened=False
    try:
        import dynamixel_sdk as sdk
        bus=Bus(sdk,report)
        if not bus.port.openPort(): raise RuntimeError('Cannot open COM7')
        opened=True
        if not bus.port.setBaudRate(3000000): raise RuntimeError('Cannot set baud')
        initial=bus.preflight()
        moves,staged,goal=plan(initial['position'])
        print('EMPTY HAND: hold ID12; prepare other 15; then test ID12 at 100mA. NOT SAC.',flush=True)
        print('IDs12/13 100mA; all others 60mA. Measured stops IDs12/13 110mA, others 90mA.',flush=True)
        print('NO intermediate torque OFF. Total timeout 60s; final target check 2s then all OFF.',flush=True)
        print('Tracking wait/resume uses each phase target tolerance; recovery deadline remains 2s.',flush=True)
        print('Cube removed; secure hand; clear fingers/cables. Q/Ctrl+C stops; cut power for contact/noise.',flush=True)
        for i,target in moves:
            print(f'Recovery ID{i+1}: {initial["position"][i]} -> {target} (<=0.15rad)',flush=True)
        print('Final targets:',goal.tolist(),flush=True)
        if input('Type POSTURE12_100 to arm: ').strip()!='POSTURE12_100':
            report['status']='cancelled_before_writes'
            return 0
        for _ in range(60):
            base.check_cancel(); time.sleep(.05)
        fresh=bus.preflight()
        fresh_moves,_,fresh_goal=plan(fresh['position'])
        if (np.any(np.abs(fresh['position']-initial['position'])>1) or
                [i for i,_ in fresh_moves]!=[i for i,_ in moves] or
                not np.array_equal(fresh_goal,goal)):
            raise RuntimeError('Pose changed after confirmation; inspect before retry')
        run_trial(bus,fresh,report)
        print('Controlled-posture ID12 target reached. All torque OFF verified. NOT SAC.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted')
        print('STOP: '+report['error'],flush=True)
    finally:
        if bus is not None and opened:
            try:
                torque=bus.block(64,1,[('torque',64,1)])['torque']
                report['final_torque_states']=torque.tolist()
                report['all_torque_off_verified']=bool(np.all(torque==0))
            except Exception as exc:
                report.update(all_torque_off_verified=False,final_check_error=str(exc))
            if not report['all_torque_off_verified']: print('CUT MOTOR POWER NOW: OFF unverified.',flush=True)
            bus.port.closePort()
        report['summary']=summarize(report)
        print('SUMMARY: '+json.dumps(report['summary']),flush=True)
        tag=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        for name in (f'hand12_posture_target_100_{tag}.json','hand12_posture_target_100_result.json'):
            path=base.ROOT/name
            path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(f'Saved: {path}\nDo not repeat automatically.',flush=True)
    return 0 if report['status']=='posture_controlled_id12_target_reached_before_off' and report.get('all_torque_off_verified') else 1


if __name__=='__main__': raise SystemExit(main())
