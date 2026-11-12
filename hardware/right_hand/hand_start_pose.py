"""Empty-hand low-current DEFAULT_POSE trial; never loads or runs a policy.

Uses the existing calibrated ID order, not the original demo's wiring comments.
Motion is explicitly armed and always followed by torque-off verification.
Control table: https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/
"""
import datetime
import json
import sys
import time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work/Leap_Test/deploy'))
from leap_hw import OFFSET, SIM2ARR, sim_to_hw, hw_to_sim
import limits
from hand_small_recovery import check_cancel

IDS = list(range(1,17))
TICK = 2*np.pi/4096
CURRENT_MA = 60
START_CURRENT_BY_ID = np.full(16, CURRENT_MA, dtype=int)
START_CURRENT_BY_ID[12] = 80  # ID13 only, matching the approved isolated trial.
START_CURRENT_BY_ID.flags.writeable = False
MAX_MOVE_RAD = 1.25
MAX_TIME_S = 60
MAX_TRACKING_ERROR = .10
PRELOAD_TOLERANCE_TICKS = 1
ENABLE_MAX_SHIFT_TICKS = 4


def current_profile(id13_current_ma=80):
    """Explicit trial choices only; never mutate the original v2 profile."""
    if id13_current_ma not in (80, 100):
        raise ValueError('Unapproved ID13 current profile')
    caps = START_CURRENT_BY_ID.copy()
    caps[12] = id13_current_ma
    stops = np.full(16, 90, dtype=int)
    if id13_current_ma == 100:
        stops[12] = 110
    return caps, stops


def validate_preload(state, requested, *, expected_current_ma=CURRENT_MA, expected_profile_velocity=5):
    """Only for torque-OFF holding initialization, never a moving goal write.

The safety invariant is a goal at the actual rest position. Accept at most one
encoder tick relative to the unchanged request, with no cumulative rebasing.
This does not establish why the register changed or that a write persisted.
"""
    if np.any(state['torque']) or np.any(state['error']) or np.any(state['watchdog']):
        raise RuntimeError('Pre-enable torque/error/watchdog state changed')
    if np.any(state['current_cap'] != expected_current_ma) or np.any(state['acceleration'] != 5) or np.any(state['velocity'] != expected_profile_velocity):
        raise RuntimeError('Pre-enable current/profile configuration changed')
    if np.any(state['temperature'] > 45) or np.any(state['voltage'] < 35) or np.any(state['voltage'] > 65):
        raise RuntimeError('Pre-enable temperature/voltage outside trial bounds')
    comparisons = [('goal-request',state['goal']-requested),
                   ('position-request',state['position']-requested),
                   ('goal-position',state['goal']-state['position'])]
    for name,delta in comparisons:
        bad = np.flatnonzero(np.abs(delta) > PRELOAD_TOLERANCE_TICKS)
        if len(bad):
            i = int(bad[0])
            raise RuntimeError(f'ID {i+1} preload {name} mismatch: request={requested[i]}, '
                               f'goal={state["goal"][i]}, actual={state["position"][i]}; limit=1 tick')
    return state['goal'].copy()


def target_plan(raw):
    raw = np.asarray(raw)
    if raw.shape != (16,) or not np.isfinite(raw).all() or np.any(raw != np.rint(raw)):
        raise ValueError('Invalid encoder array')
    if np.any(raw < 0) or np.any(raw > 4095):
        raise ValueError('Single-turn position required; no wrapping/zero changes')
    q = hw_to_sim(raw*TICK)
    # Permit the previously observed relaxed PIP/DIP extension only, and only
    # move inward from there. Never lower deployment limits to accept it.
    start_lower = limits.SAFE_LOWERS.copy()
    start_lower[[2,3,6,7,10,11]] = -.34
    if np.any(q < start_lower) or np.any(q > limits.SAFE_UPPERS):
        raise ValueError('Starting pose outside verified preparation envelope')
    goal_q = limits.DEFAULT_POSE.copy()
    if np.any(goal_q < limits.SAFE_LOWERS) or np.any(goal_q > limits.SAFE_UPPERS):
        raise ValueError('DEFAULT_POSE outside configured limits')
    goal = np.rint(sim_to_hw(goal_q)/TICK).astype(int)
    if np.max(np.abs(goal-raw))*TICK > MAX_MOVE_RAD:
        raise ValueError('Required movement exceeds 1.25 rad; no command sent')
    return goal


class SyncBus:
    read_timeout_recovery = False
    read_corrupt_recovery = False

    def __init__(self, sdk, report):
        self.sdk, self.report = sdk, report
        self.port, self.packet = sdk.PortHandler('COM7'), sdk.PacketHandler(2.0)
        self.events = report.setdefault('write_events', [])
        self._read_recovery_times = []

    def _receive(self, operation, label, request):
        """One fresh read after RX_TIMEOUT only; never repeats a servo write."""
        began=time.monotonic()
        recovery=None
        for attempt in range(2):
            value,comm,error,detail=operation()
            stamp=time.monotonic()
            if comm==self.sdk.COMM_SUCCESS and not error:
                if self.read_timeout_recovery and stamp-began>.2:
                    raise RuntimeError(f'{label}: read exceeded 200ms; no target advance')
                if recovery is not None:
                    recovery.update(recovered=True,total_elapsed_ms=(stamp-began)*1000)
                return value
            event=dict(request,**detail,attempt=attempt+1,comm=comm,device_error=error,
                       elapsed_ms=(stamp-began)*1000,retry_requested=False,recovered=False)
            self.report.setdefault('read_failures',[]).append(event)
            self._read_recovery_times=[t for t in self._read_recovery_times if stamp-t<10.]
            # Reserve at least 100ms of the existing read budget for a fresh transaction.
            retry=(self.read_timeout_recovery and
                   (comm==-3001 or (self.read_corrupt_recovery and comm==-3002))
                   and not error and attempt==0
                   and stamp-began<.1 and len(self._read_recovery_times)<3)
            if not retry:
                extra=f'; first incomplete ID {detail["first_incomplete_id"]}' if detail.get('first_incomplete_id') else ''
                raise RuntimeError(f'{label}: comm={comm}, error={error}{extra}; no target advance')
            self._read_recovery_times.append(stamp)
            event['retry_requested']=True
            recovery=event
        raise RuntimeError('Read recovery exhausted')

    def read(self, motor, addr, size=1):
        fn = {1:self.packet.read1ByteTxRx,2:self.packet.read2ByteTxRx,4:self.packet.read4ByteTxRx}[size]
        return int(self._receive(lambda:(*fn(self.port,motor,addr),{}),f'ID {motor} READ {addr}',
                                 dict(kind='read',motor_id=motor,address=addr,length=size)))

    def block(self, address, length, fields):
        return self._receive(lambda:self._block_once(address,length,fields),f'Sync read at {address} failed',
                             dict(kind='sync_read',address=address,length=length))

    def _block_once(self, address, length, fields):
        # Fresh storage for each transaction: no stale data after a failed read.
        group = self.sdk.GroupSyncRead(self.port,self.packet,address,length)
        try:
            for i in IDS:
                if not group.addParam(i):
                    raise RuntimeError(f'Cannot add ID {i} to sync read')
            comm = group.txRxPacket()
            if comm != self.sdk.COMM_SUCCESS:
                completed=[i for i,data in getattr(group,'data_dict',{}).items() if len(data)==length]
                return None,comm,0,dict(completed_ids=completed,
                    first_incomplete_id=next((i for i in IDS if i not in completed),None))
            out = {}
            for name,addr,size in fields:
                values = []
                for i in IDS:
                    if not group.isAvailable(i,addr,size):
                        raise RuntimeError(f'ID {i} data unavailable at {addr}')
                    values.append(int(group.getData(i,addr,size)))
                out[name] = np.array(values,dtype=np.int64)
            return out,self.sdk.COMM_SUCCESS,0,{}
        finally:
            group.clearParam()

    def snapshot(self):
        return self.block(64,83,[('torque',64,1),('error',70,1),('watchdog',98,1),
                                ('current_cap',102,2),('acceleration',108,4),('velocity',112,4),
                                ('goal',116,4),('current',126,2),('position',132,4),
                                ('voltage',144,2),('temperature',146,1)])

    def write(self, addr, size, values, verify=True):
        if (addr,size) not in {(64,1),(80,2),(82,2),(84,2),(98,1),(102,2),(108,4),(112,4),(116,4)}:
            raise ValueError('Unapproved register write')
        values = np.asarray(values)
        if values.shape != (16,) or np.any(values < 0) or np.any(values >= (1 << (8*size))):
            raise ValueError('Invalid sync write payload')
        event = {'address':addr,'size':size,'requested':values.astype(int).tolist()}
        self.events.append(event)
        writer = self.sdk.GroupSyncWrite(self.port,self.packet,addr,size)
        try:
            for i,value in zip(IDS,values):
                if not writer.addParam(i,list(int(value).to_bytes(size,'little'))):
                    raise RuntimeError(f'Cannot add ID {i} to sync write')
            comm = writer.txPacket()
            event['comm'] = int(comm)
            if comm != self.sdk.COMM_SUCCESS:
                raise RuntimeError(f'Sync write {addr} failed: {comm}')
        finally:
            writer.clearParam()
        if verify:
            observed = self.block(addr,size,[('value',addr,size)])['value']
            event['readback'] = observed.tolist()
            bad = np.flatnonzero(observed != values)
            if len(bad):
                i = int(bad[0])
                raise RuntimeError(f'ID {i+1} register {addr} mismatch: wrote={values[i]}, read={observed[i]}')

    def preflight(self, *, id13_current_ma=80):
        caps, _ = current_profile(id13_current_ma)
        for motor in IDS:
            model,comm,error = self.packet.ping(self.port,motor)
            if comm != self.sdk.COMM_SUCCESS or error or model != 1240:
                raise RuntimeError(f'ID {motor}: model/communication mismatch')
            for addr,value in ((64,0),(70,0),(10,0),(11,5),(98,0)):
                if self.read(motor,addr) != value:
                    raise RuntimeError(f'ID {motor}: unexpected register {addr}; no automatic changes')
            if self.read(motor,6) < 38 or self.read(motor,20,4) != 0:
                raise RuntimeError(f'ID {motor}: unsupported firmware/offset')
            if self.read(motor,38,2) < caps[motor-1]:
                raise RuntimeError(f'ID {motor}: insufficient configured current limit')
        state = self.snapshot()
        if np.any(state['torque']) or np.any(state['error']) or np.any(state['watchdog']):
            raise RuntimeError('Unexpected torque/error/watchdog state')
        if np.any(state['temperature'] > 45) or np.any(state['voltage'] < 35) or np.any(state['voltage'] > 65):
            raise RuntimeError('Temperature/voltage outside conservative trial bounds')
        target_plan(state['position'])
        return state

    def off(self):
        errors = []
        # Only emergency OFF may be retried; never retry a goal or enable.
        for _ in range(2):
            try:
                self.write(64,1,np.zeros(16,dtype=int))
                return
            except Exception as exc:
                errors.append(str(exc))
        raise RuntimeError('CUT MOTOR POWER NOW: torque OFF unverified: '+'; '.join(errors))


def validate_motion(state, start, goal, command, *, id13_current_ma=80):
    caps, stops = current_profile(id13_current_ma)
    if np.any(state['torque'] != 1) or np.any(state['error']) or np.any(state['watchdog'] != 25):
        raise RuntimeError('Torque/error/watchdog state changed')
    if np.any(state['goal'] != command):
        i = int(np.flatnonzero(state['goal'] != command)[0])
        raise RuntimeError(f'ID {i+1} goal changed: wrote={command[i]}, read={state["goal"][i]}')
    if np.any(state['current_cap'] != caps):
        raise RuntimeError('Current cap changed')
    if np.any(state['acceleration'] != 5) or np.any(state['velocity'] != 5):
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
    if np.max(errors) > MAX_TRACKING_ERROR:
        i = int(np.argmax(errors))
        raise RuntimeError(f'ID {i+1} tracking error {errors[i]:.3f}rad: '
                           f'goal={command[i]}, actual={position[i]}; no stronger retry')


def check_enable_transition(bus, start, goal, preloaded, report,
                            cancel=check_cancel, now=time.monotonic, sleep=time.sleep, *, id13_current_ma=80):
    """One bounded hold check for all IDs; not a moving-goal tolerance.

    This does not establish why a goal changed on enable. Two stable fresh
    samples must satisfy fixed pre-ON anchors and all existing health limits.
    """
    report['phase'] = 'verify_enable_transition'
    log = report['enable_transition'] = {'max_shift_ticks':ENABLE_MAX_SHIFT_TICKS,
                                        'preloaded_goal_raw':preloaded.tolist(),'samples':[]}
    began = now()
    first_goal = first_position = None
    for _ in range(2):
        cancel()
        state = bus.snapshot()
        log['samples'].append({'t':now()-began,'state_all_ids':{k:v.tolist() for k,v in state.items()}})
        if now()-began > .5:
            raise RuntimeError('Enable transition read timeout')
        observed, actual = state['goal'], state['position']
        # Validate health/path relative to the observed hold, then separately
        # restrict that hold against the unchanged initial request below.
        validate_motion(state,start,goal,observed,id13_current_ma=id13_current_ma)
        for value in (observed,actual):
            bad = np.flatnonzero((np.abs(value-start)>ENABLE_MAX_SHIFT_TICKS) |
                                 (np.abs(value-preloaded)>ENABLE_MAX_SHIFT_TICKS))
            if len(bad):
                raise RuntimeError(f'ID {int(bad[0])+1} enable shift exceeds 4 ticks; no rebasing')
        if np.any(np.abs(actual-observed)>1):
            raise RuntimeError('Enable transition goal is not at present position')
        if first_goal is None:
            first_goal, first_position = observed.copy(),actual.copy()
        elif not np.array_equal(observed,first_goal) or np.any(np.abs(actual-first_position)>1):
            raise RuntimeError('Enable transition is not stable; no retry')
        sleep(.02)
    # Recheck every joint's absolute limits and movement budget at the accepted
    # hold; never change DEFAULT_POSE or enlarge the 1.25rad travel bound.
    if not np.array_equal(target_plan(first_goal),goal):
        raise RuntimeError('Enable transition would change planned target')
    log['accepted_hold_raw'] = first_goal.tolist()
    log['shift_from_preload_ticks'] = (first_goal-preloaded).tolist()
    cancel()
    report['phase'] = 'confirm_enabled_hold'
    bus.write(116,4,first_goal)  # Exact readback, no retry; ramp stays exact too.
    print('Enable check v2 passed: 16 stable holds, strict readback. Starting slow ramp.',flush=True)
    return first_goal


def run_trial(bus, initial, report, cancel=check_cancel, now=time.monotonic, sleep=time.sleep, *, id13_current_ma=80):
    caps, stops = current_profile(id13_current_ma)
    report['current_ma_by_hw_id'] = caps.tolist()
    report['measured_current_stop_ma_by_hw_id'] = stops.tolist()
    start = initial['position'].copy()
    goal = target_plan(start)
    report['start_hw_raw'], report['goal_hw_raw'] = start.tolist(),goal.tolist()
    report['saved_ram'] = {k:initial[k].tolist() for k in ('watchdog','current_cap','acceleration','velocity')}
    touched = False
    try:
        cancel()
        if np.any(initial['torque']):
            raise RuntimeError('All motors must begin torque OFF')
        touched = True  # A failed write may still have reached one or more servos.
        report['phase'] = 'configure_torque_off'
        bus.write(102,2,caps)
        bus.write(108,4,np.full(16,5))
        bus.write(112,4,np.full(16,5))
        report['phase'] = 'preload_current_positions_torque_off'
        # Single write only. Do not generalize this tolerance to other registers
        # or to the ramp. Record two fresh snapshots against the SAME request.
        bus.write(116,4,start,verify=False)
        report['preload_snapshots'] = []
        for _ in range(2):
            state = bus.snapshot()
            report['preload_snapshots'].append({k:v.tolist() for k,v in state.items()})
            initial_command = validate_preload(state,start,expected_current_ma=caps)
            sleep(.02)
        report['preload_tolerance_ticks'] = PRELOAD_TOLERANCE_TICKS
        report['accepted_initial_goal_raw'] = initial_command.tolist()
        bus.write(98,1,np.full(16,25))
        cancel()
        report['phase'] = 'enable_at_current_position'
        bus.write(64,1,np.ones(16,dtype=int))
        enabled_at = now()
        command = check_enable_transition(bus,start,goal,initial_command,report,
                                          cancel=cancel,now=now,sleep=sleep,id13_current_ma=id13_current_ma)
        report['phase'] = 'slow_start_pose_ramp'
        began = now()
        previous, reached = began, None
        last_print = began
        report['samples'] = []
        while True:
            cancel()
            if now()-enabled_at > MAX_TIME_S:
                raise RuntimeError('60-second trial timeout')
            state = bus.snapshot()
            stamp = now()
            errors = np.abs(state['position']-command)*TICK
            worst = int(np.argmax(errors))
            currents = np.where(state['current']>=32768,state['current']-65536,state['current'])
            report['samples'].append({'t':stamp-began,'command_hw_raw':command.tolist(),
                                      'actual_hw_raw':state['position'].tolist(),
                                      'max_tracking_error_rad':float(errors[worst]),
                                      'worst_tracking_id':worst+1,
                                      'present_input_current_ma':currents.tolist(),
                                      'state_all_ids':{k:v.tolist() for k,v in state.items()}})
            if stamp-enabled_at > MAX_TIME_S:
                raise RuntimeError('60-second trial timeout after read')
            validate_motion(state,start,goal,command,id13_current_ma=id13_current_ma)
            if stamp-last_print >= 1:
                print(f'{stamp-began:.1f}s: largest tracking error ID{worst+1} '
                      f'{errors[worst]:.3f}rad, input current {currents[worst]}mA',flush=True)
                last_print = stamp
            if np.array_equal(command,goal):
                reached = stamp if reached is None else reached
                if stamp-reached >= 2:
                    if np.max(np.abs(state['position']-goal))*TICK > .035:
                        raise RuntimeError('Start pose not reached at low current')
                    report['final_sim_angles_before_off'] = hw_to_sim(state['position']*TICK).tolist()
                    report['status'] = 'start_pose_reached_before_torque_off'
                    break
            if stamp-previous >= .05 and not np.array_equal(command,goal):
                # Two ticks max (~0.0031 rad), no catch-up after delayed frames.
                command = command + np.clip(goal-command,-2,2)
                bus.write(116,4,command,verify=False)
                previous = stamp
            sleep(.01)
    except BaseException as exc:
        report['failed_phase'] = report.get('phase')
        raise
    finally:
        if touched:
            try:
                bus.off()
                report['all_torque_off_verified'] = True
            except Exception:
                report['all_torque_off_verified'] = False
                raise
            # Never restore higher current before all 16 motors are confirmed OFF.
            bus.write(98,1,np.zeros(16,dtype=int))
            for addr,size,name in ((102,2,'current_cap'),(108,4,'acceleration'),(112,4,'velocity'),(98,1,'watchdog')):
                bus.write(addr,size,initial[name])
            report['settings_restored'] = True


def main(*, id13_current_ma=80, confirmation='START_V2', result_stem='hand_start_pose', version='v2'):
    caps, stops = current_profile(id13_current_ma)
    report = {'timestamp':datetime.datetime.now().astimezone().isoformat(), 'status':'not_started',
              'policy_used':False,'current_ma_by_hw_id':caps.tolist(),
              'measured_current_stop_ma_by_hw_id':stops.tolist(),
              'all_torque_off_verified':False,'version':f'{version}_enable_check_v2_ID13_{id13_current_ma}'}
    bus = None
    opened = False
    try:
        import dynamixel_sdk as sdk
        bus = SyncBus(sdk,report)
        if not bus.port.openPort(): raise RuntimeError('Cannot open COM7')
        opened = True
        if not bus.port.setBaudRate(3000000): raise RuntimeError('Cannot set host baud')
        initial = bus.preflight(id13_current_ma=id13_current_ma)
        goal = target_plan(initial['position'])
        print(f'EMPTY HAND START POSE {version}: all 16 motors, ID13 {id13_current_ma}mA; other IDs 60mA. NOT SAC.',flush=True)
        print(f'Measured-current stop: ID13 {stops[12]}mA; others 90mA. No automatic increase or retry.',flush=True)
        print('Enable check v2: 2 stable holds, <=4 ticks from preload. Moving goal readback stays exact.',flush=True)
        print('Cube removed; hand secured; fingers/cables clear. Q/Ctrl+C stops.',flush=True)
        print('Cut motor power immediately for unexpected movement, contact, or noise.',flush=True)
        for i in range(16):
            print(f'ID {i+1:2d}: raw {initial["position"][i]} -> {goal[i]}, delta {(goal[i]-initial["position"][i])*TICK:+.3f} rad')
        if input(f'Type {confirmation} then Enter to arm this motion: ').strip() != confirmation:
            report['status'] = 'cancelled_before_writes'
            return 0
        for _ in range(60):
            check_cancel()
            time.sleep(.05)
        fresh = bus.preflight(id13_current_ma=id13_current_ma)
        if np.max(np.abs(fresh['position']-initial['position']))*TICK > .025:
            raise RuntimeError('Pose changed since confirmation; restart inspection')
        print('Preparing and moving slowly. Automatic torque OFF after a 2-second target check.',flush=True)
        run_trial(bus,fresh,report,id13_current_ma=id13_current_ma)
        print('Start pose reached; torque OFF verified. This was NOT policy control.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted')
        print('STOP: '+report['error'],flush=True)
    finally:
        if bus and opened:
            try:
                torque = bus.block(64,1,[('torque',64,1)])['torque']
                report['final_torque_states'] = torque.tolist()
                report['all_torque_off_verified'] = bool(np.all(torque == 0))
            except Exception as exc:
                report['all_torque_off_verified'] = False
                report['final_check_error'] = str(exc)
            if not report['all_torque_off_verified']:
                print('CUT MOTOR POWER NOW: torque OFF not verified.',flush=True)
            bus.port.closePort()
        tag = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        for name in [f'{result_stem}_{tag}.json',f'{result_stem}_result.json']:
            path = ROOT/name
            path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(f'Saved: {path}',flush=True)
        print('Fingers may relax after torque OFF; do not rerun automatically.',flush=True)
    return 0 if report['status'] == 'start_pose_reached_before_torque_off' and report['all_torque_off_verified'] else 1


if __name__ == '__main__': raise SystemExit(main())
