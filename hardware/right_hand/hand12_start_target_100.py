"""Armed ID12-only diagnostic. No policy, EEPROM, mode, zero or gain writes.

XC330-M288 register reference:
https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/
Present Current is input-current telemetry, not a calibrated contact-force sensor.
"""
import datetime
import json
import time
import numpy as np
from hand_start_pose import (SyncBus, ROOT, TICK,
                             validate_preload, check_cancel)
from leap_hw import hw_to_sim, sim_to_hw, SIM2ARR
import limits

MOTOR = 12
INDEX = 11
MAX_MOVE = .45
CURRENT_MA = 100  # Explicit ID12-only 80 -> 100mA one-shot trial.
ENABLE_MAX_SHIFT_TICKS = 4  # ~0.35 deg; one transition only, NOT ramp tolerance.


def require_current_profile(current_ma, current_stop_ma):
    # Only explicitly prepared profiles; never auto-tune or increase on failure.
    if (current_ma,current_stop_ma) not in ((100,110),):
        raise ValueError('Unapproved current profile; allowed cap/stop pairs: 100/110mA only')


class SingleBus(SyncBus):
    current_ma = CURRENT_MA

    def __init__(self,sdk,report,*,current_ma=CURRENT_MA,current_stop_ma=110):
        require_current_profile(current_ma,current_stop_ma)
        self.current_ma = current_ma
        super().__init__(sdk,report)

    def preflight(self):
        # This is NOT a whole-hand start-pose test. Never require or command
        # the other fifteen torque-OFF joints to reach a preparation pose.
        rows = self.report['preflight_motors'] = []
        for motor in range(1,17):
            row = {'id':motor}
            rows.append(row)
            model,comm,error = self.packet.ping(self.port,motor)
            row.update(model=int(model),comm=int(comm),device_error=int(error))
            if comm != self.sdk.COMM_SUCCESS or error or model != 1240:
                raise RuntimeError(f'ID {motor}: model/communication mismatch: {row}')
            for addr,size,expected in ((64,1,0),(70,1,0),(10,1,0),(11,1,5),(98,1,0),(20,4,0)):
                value = self.read(motor,addr,size)
                row[str(addr)] = value
                if value != expected:
                    raise RuntimeError(f'ID {motor}: register {addr} expected={expected}, actual={value}; no correction')
            row['firmware'] = self.read(motor,6)
            if row['firmware'] < 38:
                raise RuntimeError(f'ID {motor}: unsupported firmware {row["firmware"]}')
        state = self.snapshot()
        self.report['preflight_state_all_ids'] = pack(state)
        if np.any(state['torque']) or np.any(state['error']) or np.any(state['watchdog']):
            raise RuntimeError('All 16 must be torque OFF without hardware error/watchdog; see saved snapshot')
        bad=np.flatnonzero((state['temperature']>45)|(state['voltage']<35)|(state['voltage']>65))
        if len(bad):
            i=int(bad[0])
            raise RuntimeError(f'ID {i+1}: voltage={state["voltage"][i]/10:.1f}V, temperature={state["temperature"][i]}C outside trial bounds')
        if self.read(MOTOR,38,2) < self.current_ma:
            raise RuntimeError(f'ID12 configured current limit below {self.current_ma}mA; no correction')
        make_target(state['position'])
        q=hw_to_sim(state['position']*TICK)
        outside=np.flatnonzero((q<limits.SAFE_LOWERS)|(q>limits.SAFE_UPPERS))
        self.report['nonmoving_outside_policy_ids']=[int(SIM2ARR[i])+1 for i in outside if int(SIM2ARR[i])+1 != MOTOR]
        return state

    def write(self, addr, size, value, verify=True):
        # Unicast only: no method here can enable/write targets to another ID.
        if (addr, size) not in {(64,1),(98,1),(102,2),(108,4),(112,4),(116,4)}:
            raise ValueError('Unapproved register')
        if not isinstance(value, (int, np.integer)) or not 0 <= value < 2**(8*size):
            raise ValueError('Scalar integer payload required')
        event = {'motor_id': MOTOR, 'address': addr, 'requested': int(value)}
        self.events.append(event)
        fn = {1:self.packet.write1ByteTxRx, 2:self.packet.write2ByteTxRx,
              4:self.packet.write4ByteTxRx}[size]
        comm, error = fn(self.port, MOTOR, addr, int(value))
        event.update(comm=int(comm), device_error=int(error))
        if comm != self.sdk.COMM_SUCCESS or error:
            raise RuntimeError(f'ID12 write {addr}: comm={comm}, error={error}')
        if verify:
            observed = self.read(MOTOR, addr, size)
            event['readback'] = observed
            if observed != value:
                raise RuntimeError(f'ID12 register {addr}: wrote={value}, read={observed}')

    def off(self):
        errors = []
        for _ in range(2):  # OFF only may be retried, never goal/enable.
            try:
                self.write(64, 1, 0)
                torque = self.block(64, 1, [('torque',64,1)])['torque']
                if np.any(torque):
                    raise RuntimeError('Another motor is enabled')
                return
            except Exception as exc:
                errors.append(str(exc))
        raise RuntimeError('CUT MOTOR POWER: torque OFF unverified: '+ '; '.join(errors))


def make_target(raw):
    raw=np.asarray(raw)
    if raw.shape!=(16,) or not np.isfinite(raw).all() or np.any(raw!=np.rint(raw)):
        raise ValueError('Invalid encoder array; no movement')
    bad=np.flatnonzero((raw<0)|(raw>4095))
    if len(bad):
        i=int(bad[0])
        raise ValueError(f'ID {i+1}: invalid single-turn encoder raw={raw[i]}; no wrapping')
    sim_index=int(np.flatnonzero(SIM2ARR==INDEX)[0])
    angle=float(hw_to_sim(raw*TICK)[sim_index])
    low,high=float(limits.SAFE_LOWERS[sim_index]),float(limits.SAFE_UPPERS[sim_index])
    if not low <= angle <= high:
        raise ValueError(f'ID12 raw={raw[INDEX]}, angle={angle:+.4f}rad outside [{low},{high}]; no movement')
    if not low <= limits.DEFAULT_POSE[sim_index] <= high:
        raise ValueError('ID12 configured target outside policy bounds')
    target = int(np.rint(sim_to_hw(limits.DEFAULT_POSE)[INDEX]/TICK))
    start = int(raw[INDEX])
    if target != 2570:
        raise ValueError('Configured ID12 target changed; no automatic adaptation')
    if start >= target:
        raise ValueError(f'ID12 raw={start} already at/above target={target}; no reverse test')
    if target-start+ENABLE_MAX_SHIFT_TICKS > int(np.floor(MAX_MOVE/TICK)):
        raise ValueError(f'ID12 raw={start} -> {target}, delta={(target-start)*TICK:.4f}rad exceeds 0.45rad travel budget including enable margin; no movement')
    return target


def pack(state):
    return {k: np.asarray(v).tolist() for k,v in state.items()}


def signed_current(value):
    return int(value)-65536 if value >= 32768 else int(value)


def validate(state, start, goal, command, *, current_ma=CURRENT_MA,current_stop_ma=110):
    require_current_profile(current_ma,current_stop_ma)
    expected = np.zeros(16, dtype=int)
    expected[INDEX] = 1
    if not np.array_equal(state['torque'], expected) or np.any(state['error']):
        raise RuntimeError('Torque/error changed; only ID12 may be enabled')
    for name, value in [('watchdog',25),('current_cap',current_ma),('acceleration',5),('velocity',5),('goal',command)]:
        if state[name][INDEX] != value:
            raise RuntimeError(f'ID12 {name}: expected={value}, read={state[name][INDEX]}')
    if np.any(state['temperature'] > 45) or np.any(state['voltage'] < 35) or np.any(state['voltage'] > 65):
        raise RuntimeError('Temperature/voltage outside test bounds')
    if abs(signed_current(state['current'][INDEX])) > current_stop_ma:
        raise RuntimeError(f'ID12 measured input current exceeds {current_stop_ma}mA stop threshold')
    actual = int(state['position'][INDEX])
    if not start-15 <= actual <= goal+15:
        raise RuntimeError('ID12 outside planned path')
    if abs(actual-command)*TICK > .075:
        raise RuntimeError('ID12 tracking error >0.075rad; no stronger retry')


def check_enable_transition(bus, start, goal, preloaded, report,
                            cancel=check_cancel, now=time.monotonic, sleep=time.sleep,
                            *, max_move=MAX_MOVE,current_ma=CURRENT_MA,current_stop_ma=110):
    """Validate a stationary hold after ON; does not prove why a goal changed.

    No retry, growing tolerance, encoder offset change, or total-range extension.
    Both samples refer to the same pre-ON positions, never to a moving baseline.
    """
    report['phase'] = 'verify_enable_transition'
    report['enable_transition'] = {'max_shift_ticks':ENABLE_MAX_SHIFT_TICKS,
                                   'preloaded_goal_raw':preloaded,'samples':[]}
    log = report['enable_transition']
    began = now()
    first_goal = first_position = None
    for _ in range(2):
        cancel()
        state = bus.snapshot()
        actual = int(state['position'][INDEX])
        observed = int(state['goal'][INDEX])
        log['samples'].append({'t':now()-began, 'state_all_ids':pack(state)})
        if now()-began > .5:
            raise RuntimeError('Enable transition read timeout')
        # Reuse health/path checks with the observed hold, THEN independently
        # bound that hold against the fixed original request below.
        validate(state,start,goal,observed,current_ma=current_ma,current_stop_ma=current_stop_ma)
        for value in (actual,observed):
            if abs(value-start) > ENABLE_MAX_SHIFT_TICKS or abs(value-preloaded) > ENABLE_MAX_SHIFT_TICKS:
                raise RuntimeError('Enable transition shift exceeds 4 ticks; no rebasing')
        if abs(actual-observed) > 1:
            raise RuntimeError('Enable transition goal is not at present position')
        if first_goal is None:
            first_goal,first_position = observed,actual
        elif observed != first_goal or abs(actual-first_position) > 1:
            raise RuntimeError('Enable transition is not stable; no retry')
        sleep(.02)
    # Retain the original upper goal; negative shifts cannot extend total travel.
    bounded_goal = min(goal, first_goal + int(np.floor(max_move/TICK)))
    if first_goal >= bounded_goal:
        raise RuntimeError('No positive test travel after enable; no reverse command')
    log.update(accepted_hold_raw=first_goal, bounded_target_raw=bounded_goal,
               shift_from_preload_ticks=first_goal-preloaded)
    cancel()
    report['phase'] = 'confirm_enabled_hold'
    # Explicitly write this stationary hold once, with exact readback. After
    # this point ALL goal comparisons are strict, even a one-tick change fails.
    bus.write(116,4,first_goal)
    print(f'Enable check: preload={preloaded}, hold={first_goal}; '
          '2 stable samples, strict hold readback. Starting small ramp.',flush=True)
    return first_goal,bounded_goal


def run_trial(bus, initial, report, cancel=check_cancel, now=time.monotonic, sleep=time.sleep,
              *, planner=make_target, max_move=MAX_MOVE, max_time=25,
              current_ma=CURRENT_MA,current_stop_ma=110):
    require_current_profile(current_ma,current_stop_ma)
    start = int(initial['position'][INDEX])
    goal = planner(initial['position'])
    if not 0 < (goal-start)*TICK <= max_move:
        raise ValueError('Planned movement outside diagnostic travel bound')
    report.update(start_raw=start, target_raw=goal, initial_state=pack(initial), samples=[],
                  current_ma=current_ma,measured_current_stop_ma=current_stop_ma)
    touched = False
    try:
        cancel()
        if np.any(initial['torque']):
            raise RuntimeError('All motors must begin torque OFF')
        touched = True
        report['phase'] = 'configure_torque_off'
        for addr,size,value in [(102,2,current_ma),(108,4,5),(112,4,5)]:
            bus.write(addr,size,value)
        report['phase'] = 'preload_torque_off'
        bus.write(116,4,start,verify=False)
        report['preload_snapshots'] = []
        for _ in range(2):
            state = bus.snapshot()
            report['preload_snapshots'].append(pack(state))
            if np.any(state['torque']) or np.any(state['error']):
                raise RuntimeError('Unexpected torque/error before enable')
            command = int(validate_preload({k:v[INDEX:INDEX+1] for k,v in state.items()},
                                           np.array([start]), expected_current_ma=current_ma)[0])
            sleep(.02)
        bus.write(98,1,25)
        cancel()
        report['phase'] = 'enable_ID12_only'
        bus.write(64,1,1)
        enabled_at = now()
        command,goal = check_enable_transition(bus,start,goal,command,report,
                                               cancel=cancel,now=now,sleep=sleep,max_move=max_move,
                                               current_ma=current_ma,current_stop_ma=current_stop_ma)
        report['effective_target_raw'] = goal
        report['phase'] = 'small_ramp_ID12_only'
        began = previous = last_print = now()
        reached = None
        while True:
            cancel()
            if now()-enabled_at > max_time:
                raise RuntimeError(f'{max_time}-second diagnostic timeout')
            state = bus.snapshot()
            stamp = now()
            sample = {'t':stamp-began, 'command_raw':command,
                      'actual_raw':int(state['position'][INDEX]),
                      'tracking_error_rad':float((command-state['position'][INDEX])*TICK),
                      'present_input_current_ma':signed_current(state['current'][INDEX]),
                      'voltage_v':float(state['voltage'][INDEX])*.1,
                      'temperature_c':int(state['temperature'][INDEX]),
                      'state_all_ids':pack(state)}
            # Keep the failed sample too; validation must follow logging.
            report['samples'].append(sample)
            if stamp-enabled_at > max_time:
                raise RuntimeError(f'{max_time}-second diagnostic timeout after read')
            validate(state,start,goal,command,current_ma=current_ma,current_stop_ma=current_stop_ma)
            if stamp-last_print >= .5:
                print(f'ID12: actual={sample["actual_raw"]}, goal={command}, '
                      f'error={sample["tracking_error_rad"]:+.3f}rad, '
                      f'input current={sample["present_input_current_ma"]}mA',flush=True)
                last_print = stamp
            if command == goal:
                reached = stamp if reached is None else reached
                if stamp-reached >= 1:
                    if abs(sample['tracking_error_rad']) > .02:
                        raise RuntimeError('Small target not reached; no stronger retry')
                    report['status'] = 'small_target_reached_not_policy_test'
                    break
            if stamp-previous >= .05 and command != goal:
                command = min(command+2, goal)
                bus.write(116,4,command,verify=False)
                previous = stamp  # No catch-up after a slow read.
            sleep(.01)
    except BaseException as exc:
        report.update(failed_phase=report.get('phase'), error=str(exc) or type(exc).__name__)
        raise
    finally:
        if touched:
            report['all_torque_off_verified'] = False
            bus.off()
            report['all_torque_off_verified'] = True
            bus.write(98,1,0)
            # Restore only ID12 RAM, and only after all motors are verified OFF.
            for addr,size,name in [(102,2,'current_cap'),(108,4,'acceleration'),(112,4,'velocity'),(98,1,'watchdog')]:
                bus.write(addr,size,int(initial[name][INDEX]))
            report['settings_restored'] = True


def summarize(report):
    rows=report.get('samples',[])
    out={'status':report.get('status'), 'all_torque_off_verified':report.get('all_torque_off_verified',False),
         'settings_restored':report.get('settings_restored',False),
         'error':report.get('error'), 'automatic_retry':False}
    if rows:
        currents=[abs(x['present_input_current_ma']) for x in rows]
        last=rows[-1]
        out.update(actual_start_raw=report['start_raw'],actual_end_raw=last['actual_raw'],
            target_raw=report.get('effective_target_raw',report['target_raw']),
            observed_travel_rad=(last['actual_raw']-report['start_raw'])*TICK,
            last_tracking_error_rad=last['tracking_error_rad'],peak_abs_input_current_ma=max(currents),
            sampled_current_near_cap=bool(max(currents)>=report['current_ma']-2),
            min_voltage_v=min(x['voltage_v'] for x in rows),
            max_temperature_c=max(x['temperature_c'] for x in rows),
            note='Telemetry does not by itself establish the cause of load or nonmovement.')
    return out


def main(*, planner=make_target, max_move=MAX_MOVE, max_time=25,
         confirmation='TARGET12_100', result_stem='hand12_start_target_100', trial_note='ID12 ONLY: same raw2570 target; authorized 80 -> 100mA single trial. No further increase or retries.',
         current_ma=CURRENT_MA,current_stop_ma=110):
    require_current_profile(current_ma,current_stop_ma)
    report = {'timestamp':datetime.datetime.now().astimezone().isoformat(),
              'status':'not_started','policy_used':False,'motor_id':12,
              'inspection_scope':'ID12 angle/travel; all 16 communication, torque-OFF and health',
              'current_ma':current_ma,'measured_current_stop_ma':current_stop_ma,
              'max_move_rad':max_move,'max_time_s':max_time,
              'trial_note':trial_note}
    bus, opened = None, False
    try:
        import dynamixel_sdk as sdk
        bus = SingleBus(sdk,report,current_ma=current_ma,current_stop_ma=current_stop_ma)
        if not bus.port.openPort(): raise RuntimeError('Cannot open COM7; close other motor programs')
        opened = True
        if not bus.port.setBaudRate(3000000): raise RuntimeError('Cannot set host baud')
        initial = bus.preflight()
        goal = planner(initial['position'])
        print(f'EMPTY HAND: ID12 middle-finger fingertip joint (DIP) ONLY. {current_ma}mA. NOT SAC.',flush=True)
        print(trial_note,flush=True)
        print('Angle/travel gate: ID12 ONLY. Other 15 remain OFF with communication/health checks.',flush=True)
        print('Unmoved IDs outside policy range:',report.get('nonmoving_outside_policy_ids',[]),flush=True)
        print(f'{current_ma}mA cap; measured-current stop {current_stop_ma}mA; tracking stop 0.075rad; timeout {max_time}s.',flush=True)
        print('Enable check v2: two stable hold samples, <=4 ticks from preload; ramp checks stay exact.',flush=True)
        print('Other 15 motors remain torque OFF. Secure hand; remove cube; clear cables/fingers.',flush=True)
        print('Q/Ctrl+C stops; cut motor power immediately for unexpected motion/contact/noise.',flush=True)
        print(f'ID12 raw {initial["position"][INDEX]} -> {goal}; delta '
              f'{(goal-initial["position"][INDEX])*TICK:+.3f}rad (maximum {max_move:.2f}rad).',flush=True)
        if input(f'Type {confirmation} then Enter to arm: ').strip() != confirmation:
            report['status'] = 'cancelled_before_writes'
            return 0
        print('Starting in 3 seconds. Q stops.',flush=True)
        for _ in range(60):
            check_cancel()
            time.sleep(.05)
        fresh = bus.preflight()
        if abs(int(fresh['position'][INDEX])-int(initial['position'][INDEX]))*TICK > .025:
            raise RuntimeError(f'ID12 changed after confirmation: {initial["position"][INDEX]} -> {fresh["position"][INDEX]}; no movement')
        run_trial(bus,fresh,report,planner=planner,max_move=max_move,max_time=max_time,
                  current_ma=current_ma,current_stop_ma=current_stop_ma)
        print('Small diagnostic completed; torque OFF. This does not clear the full start pose.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted')
        print('STOP: '+report['error'],flush=True)
    finally:
        if bus is not None and opened:
            try:
                states = bus.block(64,1,[('torque',64,1)])['torque']
                report['final_torque_states'] = states.tolist()
                report['all_torque_off_verified'] = bool(np.all(states == 0))
            except Exception as exc:
                report.update(all_torque_off_verified=False,final_check_error=str(exc))
            if not report.get('all_torque_off_verified'):
                print('CUT MOTOR POWER NOW: torque OFF not verified.',flush=True)
            bus.port.closePort()
        report['summary'] = summarize(report)
        print('SUMMARY: '+json.dumps(report['summary']),flush=True)
        tag = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        for name in [f'{result_stem}_{tag}.json',f'{result_stem}_result.json']:
            path = ROOT/name
            path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(f'Saved: {path}',flush=True)
        print('Do not automatically repeat or raise current. Send the result for review.',flush=True)
    return 0 if report['status']=='small_target_reached_not_policy_test' and report.get('all_torque_off_verified') else 1


if __name__ == '__main__': raise SystemExit(main())
