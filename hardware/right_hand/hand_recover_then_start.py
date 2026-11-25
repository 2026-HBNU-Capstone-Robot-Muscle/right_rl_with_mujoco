"""Explicit empty-hand recovery -> start-pose test, with no intermediate OFF.
NOT policy control. Original policy and preparation limits are unchanged.
"""
import datetime
import json
import time
import numpy as np
import hand_start_pose as base
from hand_pip_inward_recovery import recovery_envelope

TICK = base.TICK
CAPS, STOPS = base.current_profile(100)


def plan(raw):
    recovery_envelope(raw)
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
    def preflight(self):
        for motor in base.IDS:
            model,comm,error = self.packet.ping(self.port,motor)
            if comm != self.sdk.COMM_SUCCESS or error or model != 1240:
                raise RuntimeError(f'ID {motor}: model/communication mismatch')
            for addr,value in ((64,0),(70,0),(10,0),(11,5),(98,0)):
                if self.read(motor,addr) != value:
                    raise RuntimeError(f'ID {motor}: unexpected register {addr}')
            if self.read(motor,6)<38 or self.read(motor,20,4)!=0:
                raise RuntimeError(f'ID {motor}: firmware/offset mismatch')
            if self.read(motor,38,2)<CAPS[motor-1]:
                raise RuntimeError(f'ID {motor}: current limit below trial cap')
        state = self.snapshot()
        if np.any(state['torque']) or np.any(state['error']) or np.any(state['watchdog']):
            raise RuntimeError('All motors must begin OFF without error/watchdog')
        if np.any(state['temperature']>45) or np.any(state['voltage']<35) or np.any(state['voltage']>65):
            raise RuntimeError('Temperature/voltage outside trial bounds')
        self.report['preflight_state'] = pack(state)
        plan(state['position'])
        return state


def pack(state):
    return {k:v.tolist() for k,v in state.items()}


def run_trial(bus, initial, report, cancel=base.check_cancel, now=time.monotonic, sleep=time.sleep, *, held_check=None):
    original = initial['position'].copy()
    moves, staged, goal = plan(original)
    report.update(initial_state=pack(initial), recovery_moves=[list(x) for x in moves],
                  goal_hw_raw=goal.tolist(), samples=[], current_ma_by_hw_id=CAPS.tolist(),
                  measured_current_stop_ma_by_hw_id=STOPS.tolist())
    touched = False
    began = None

    def sample(command, path_start, path_goal, threshold=.10):
        cancel()
        if now()-began>60:
            raise RuntimeError('60-second total torque-on timeout')
        state = bus.snapshot()
        err = np.abs(state['position']-command)*TICK
        worst = int(np.argmax(err))
        report['samples'].append(dict(t=now()-began, phase=report['phase'],
            command_hw_raw=command.tolist(), state_all_ids=pack(state),
            worst_tracking_id=worst+1, max_tracking_error_rad=float(err[worst])))
        if now()-began>60:
            raise RuntimeError('60-second timeout after read')
        base.validate_motion(state,path_start,path_goal,command,id13_current_ma=100)
        if np.max(err)>threshold:
            raise RuntimeError(f'ID {worst+1} phase tracking error {err[worst]:.3f}rad')
        return state

    def ramp(command, target, *, phase, timeout, threshold, settle, final_error, moving_index=None):
        report['phase'] = phase
        start = command.copy()
        phase_began = previous = last_print = now()
        reached = None
        while True:
            state = sample(command,start,target,threshold)
            stamp = now()
            if stamp-phase_began>timeout:
                raise RuntimeError(f'{phase}: phase timeout')
            if moving_index is not None:
                hold_errors = np.delete(np.abs(state['position']-start)*TICK,moving_index)
                if np.max(hold_errors)>.035:
                    raise RuntimeError('A held joint drifted >0.035rad during recovery')
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
            if stamp-previous>=.05 and not np.array_equal(command,target):
                command=command+np.clip(target-command,-2,2)
                bus.write(116,4,command,verify=False)
                previous=stamp
            sleep(.01)

    try:
        cancel()
        if np.any(initial['torque']):
            raise RuntimeError('All motors must begin torque OFF')
        touched=True
        report['phase']='configure_preload_OFF'
        for addr,size,values in ((102,2,CAPS),(108,4,np.full(16,5)),(112,4,np.full(16,5))):
            bus.write(addr,size,values)
        bus.write(116,4,original,verify=False)
        report['preload_samples']=[]
        for _ in range(2):
            state=bus.snapshot()
            report['preload_samples'].append(pack(state))
            preloaded=base.validate_preload(state,original,expected_current_ma=CAPS)
            sleep(.02)
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
            base.validate_motion(state,original,original,observed,id13_current_ma=100)
            for value in (observed,state['position']):
                if np.any(np.abs(value-original)>4) or np.any(np.abs(value-preloaded)>4):
                    raise RuntimeError('Enable shift exceeds 4 ticks')
            if np.any(np.abs(state['position']-observed)>1):
                raise RuntimeError('Enable hold differs from actual position')
            if first is not None and (not np.array_equal(observed,first['goal']) or
                    np.any(np.abs(state['position']-first['position'])>1)):
                raise RuntimeError('Unstable enable hold')
            first=state
            sleep(.02)
        command=first['goal'].copy()
        # Keep original targets; also budget each recovery from the accepted hold.
        for i,target in moves:
            if not 0<(target-command[i])*TICK<=.15:
                raise RuntimeError('Recovery travel exceeds bounded enable margin')
        bus.write(116,4,command)
        for i,target in moves:
            next_target=command.copy(); next_target[i]=target
            command,state=ramp(command,next_target,phase=f'recover_ID{i+1}',timeout=12,
                threshold=.075,settle=1,final_error=.02,moving_index=i)
        # Fresh measured angles, not just commands, must pass the ORIGINAL planner.
        state=sample(command,command,command,.035)
        if not np.array_equal(base.target_plan(state['position']),goal):
            raise RuntimeError('Measured recovery pose does not match start target')
        if not np.array_equal(base.target_plan(command),goal):
            raise RuntimeError('Commanded recovery pose does not match start target')
        report['recovered_state_before_start']=pack(state)
        command,state=ramp(command,goal,phase='full_start_pose',timeout=45,
            threshold=.10,settle=2,final_error=.035)
        report['final_sim_angles_before_off']=base.hw_to_sim(state['position']*TICK).tolist()
        if held_check is not None:
            report['phase']='fixed_start_pose_check'
            # Callback receives checked reads ONLY, not the bus or a goal writer.
            # The existing total 60s torque-on deadline also covers this phase.
            held_check(lambda: sample(goal,goal,goal,.035))
            sample(goal,goal,goal,.035)
        report['status']='start_pose_reached_before_torque_off'
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
            report['settings_restored']=True


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
        print('EMPTY HAND: all 16 motors HOLD, then bounded recovery, then START POSE. NOT SAC.',flush=True)
        print('ID13 100mA; all others 60mA. Measured stops ID13 110mA, others 90mA.',flush=True)
        print('NO intermediate torque OFF. Total timeout 60s; final target check 2s then all OFF.',flush=True)
        print('Cube removed; secure hand; clear fingers/cables. Q/Ctrl+C stops; cut power for contact/noise.',flush=True)
        for i,target in moves:
            print(f'Recovery ID{i+1}: {initial["position"][i]} -> {target} (<=0.15rad)',flush=True)
        print('Final targets:',goal.tolist(),flush=True)
        if input('Type HOLD_START to arm: ').strip()!='HOLD_START':
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
        print('Start pose reached. All torque OFF verified. NOT SAC.',flush=True)
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
        tag=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        for name in (f'hand_recover_then_start_{tag}.json','hand_recover_then_start_result.json'):
            path=base.ROOT/name
            path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(f'Saved: {path}\nDo not repeat automatically.',flush=True)
    return 0 if report['status']=='start_pose_reached_before_torque_off' and report.get('all_torque_off_verified') else 1


if __name__=='__main__': raise SystemExit(main())
