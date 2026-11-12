"""Checked, single-pass copy of sequence_flip.py's seven recorded poses.

The original LeapNode initialization is never run. No motor port is opened
by --check or by importing this module.
"""
import argparse
import datetime as dt
import hashlib
import json
import time
from pathlib import Path

import numpy as np
from goal_readback_confirmation import GoalReadbackMismatch

from hand_prepare_full import Bus as FullBus
from hand_small_recovery import check_cancel

ROOT = Path(__file__).resolve().parent
SOURCE = Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py')
POSES_FILE = ROOT / 'sequence_flip_poses.json'
POSES_SHA256 = 'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b'
TICK = 2 * np.pi / 4096
IDS = tuple(range(1, 17))
CAP = np.full(16, 450, dtype=int)
CURRENT_TRIP_MA = 550
MIN_INPUT_VOLTAGE_TENTHS = 37  # XC330-M288 operating minimum: 3.7 V.
# One-quantum dip: stop advancing goals and confirm with fresh full-health reads.
# These are software debounce settings, not a manufacturer undervoltage allowance.
VOLTAGE_CONFIRM_S = .15
VOLTAGE_RECOVER_TENTHS = 38
VOLTAGE_RECOVER_READS = 2
VOLTAGE_CONFIRM_MAX_READS = 6
VOLTAGE_REPEAT_WINDOW_S = 2.
VOLTAGE_REPEAT_TRIP = 3
STEPS = 70
DELAY_S = .03
SAMPLE_EVERY_STEPS = 5
MAX_PHASE_S = 60
MAX_TRACKING_RAD = .18
GAINS = {'p_gain': np.full(16,600,dtype=int),
         'i_gain': np.zeros(16,dtype=int),
         'd_gain': np.full(16,200,dtype=int)}
GAINS['p_gain'][[7,11,15]] = 450
GAINS['d_gain'][[7,11,15]] = 150


def validate_raw(raw):
    raw = np.asarray(raw)
    if (raw.shape != (16,) or not np.isfinite(raw).all() or
            np.any(raw != np.rint(raw)) or np.any(raw < 0) or np.any(raw > 4095)):
        raise ValueError('Invalid single-turn hand position')
    return raw.astype(int)


def load_sequence(source=POSES_FILE):
    data = source.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if source == POSES_FILE and digest != POSES_SHA256:
        raise ValueError('Recorded sequence file changed; verify before motor motion')
    document = json.loads(data.decode('utf-8-sig'))
    if document.get('format') != 'leap_sequence_flip_raw_ticks_v1':
        raise ValueError('Wrong sequence data format')
    poses = [(item['name'], validate_raw(item['raw_ticks'])) for item in document['poses']]
    if any(not isinstance(name,str) for name,_ in poses):
        raise ValueError('Invalid stage name')
    if len(poses) != 7:
        raise ValueError(f'Expected 7 recorded poses; got {len(poses)}')
    return digest, poses


class Bus(FullBus):
    planner = staticmethod(validate_raw)
    current_caps = CAP
    read_corrupt_recovery = True  # One fresh read only; no goal/torque write retry.

    def read(self, motor, addr, size=1):
        value = super().read(motor, addr, size)
        # Static preflight registers occasionally returned a wrong value after
        # successful communication. Confirm an anomaly with two fresh reads
        # before this runner enables any motor; never change device settings.
        expected = {6: ('firmware', lambda v: v >= 38),
                    20: ('homing_offset', lambda v: v == 0)}.get(addr)
        if expected is None or expected[1](value):
            return value
        name, valid = expected
        observed = [value]
        for _ in range(2):
            observed.append(super().read(motor, addr, size))
        event = dict(motor_id=motor, address=addr, register=name, values=observed,
                     recovered=all(valid(v) for v in observed[1:]))
        self.report.setdefault('preflight_static_read_anomalies', []).append(event)
        if not event['recovered']:
            raise RuntimeError(f'ID {motor} {name} mismatch: reads={observed}; no settings changed')
        return observed[-1]


def signed_current(raw):
    return np.where(raw >= 32768, raw - 65536, raw)


def validate_control_profile(profile):
    """Validate the optional bounded diagnostic profile before any writes."""
    if not profile:return {}
    if set(profile)!= {'caps','gains','current_trip_ma'} or set(profile['gains'])!=set(GAINS):
        raise ValueError('Invalid diagnostic control profile fields')
    def vector(values,low,high):
        a=np.asarray(values)
        if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)) or np.any(a<low) or np.any(a>high):
            raise ValueError('Diagnostic profile outside reviewed bounds')
        return a.astype(int).copy()
    caps=vector(profile['caps'],1,CAP)
    gains={k:vector(profile['gains'][k],0,{'p_gain':600,'i_gain':0,'d_gain':200}[k]) for k in GAINS}
    if np.any(gains['p_gain']==0):raise ValueError('Diagnostic P gain must be positive')
    return dict(caps=caps,gains=gains,current_trip_ma=vector(profile['current_trip_ma'],caps,CURRENT_TRIP_MA))


class MildUndervoltage(RuntimeError):
    """A 3.6V reading; only sample() may perform bounded fresh confirmation."""


def health(state, command, caps=CAP, *, gains=None, current_trip_ma=CURRENT_TRIP_MA):
    gains = GAINS if gains is None else gains
    if np.any(state['torque'] != 1) or np.any(state['error']) or np.any(state['watchdog'] != 25):
        bad = [f'ID{i+1} torque={state["torque"][i]} error={state["error"][i]} watchdog={state["watchdog"][i]}'
               for i in range(16) if state['torque'][i] != 1 or state['error'][i] or state['watchdog'][i] != 25]
        raise RuntimeError('Motor state changed: ' + '; '.join(bad))
    if (np.any(state['current_cap'] != caps) or np.any(state['velocity'] != 0)
            or np.any(state['acceleration'] != 0)):
        raise RuntimeError('Current/profile setting changed')
    for key, expected in gains.items():
        if np.any(state[key] != expected):
            raise RuntimeError(f'{key} changed during motion')
    voltage = np.asarray(state['voltage'])
    temperature = np.asarray(state['temperature'])
    if voltage.shape != (16,) or not np.isfinite(voltage).all() or np.any(voltage != np.rint(voltage)):
        raise RuntimeError('Invalid voltage telemetry; no target advance')
    if temperature.shape != (16,) or not np.isfinite(temperature).all():
        raise RuntimeError('Invalid temperature telemetry; no target advance')
    if np.any(voltage > 65):
        i = int(np.argmax(voltage))
        raise RuntimeError(f'Overvoltage: ID{i+1} {voltage[i]/10:.1f}V')
    if np.any(temperature > 45):
        i = int(np.argmax(temperature))
        raise RuntimeError(f'Overtemperature: ID{i+1} {temperature[i]}C')
    currents = signed_current(state['current'])
    trips = np.broadcast_to(current_trip_ma,(16,))
    if np.any(np.abs(currents) > trips):
        i = int(np.flatnonzero(np.abs(currents) > trips)[0])
        raise RuntimeError(f'ID{i+1} measured current {currents[i]}mA exceeded {trips[i]}mA')
    errors = np.abs(state['position'] - command) * TICK
    if np.max(errors) > MAX_TRACKING_RAD:
        i = int(np.argmax(errors))
        raise RuntimeError(f'ID{i+1} tracking {errors[i]:.3f}rad; goal={command[i]}, actual={state["position"][i]}, current={currents[i]}mA')
    if np.any(voltage < MIN_INPUT_VOLTAGE_TENTHS - 1):
        i = int(np.argmin(voltage))
        raise RuntimeError(f'Undervoltage: ID{i+1} {voltage[i]/10:.1f}V; immediate stop')
    if np.any(state['goal'] != command):
        i = int(np.flatnonzero(state['goal'] != command)[0])
        message = f'ID{i+1} goal readback {state["goal"][i]} != command {command[i]}'
        if np.any(voltage < MIN_INPUT_VOLTAGE_TENTHS):
            raise RuntimeError(message + '; simultaneous undervoltage; OFF')
        raise GoalReadbackMismatch(message)
    if np.any(voltage < MIN_INPUT_VOLTAGE_TENTHS):
        i = int(np.argmin(voltage))
        message = f'Undervoltage: ID{i+1} {voltage[i]/10:.1f}V'
        if voltage[i] == MIN_INPUT_VOLTAGE_TENTHS - 1:
            raise MildUndervoltage(message)
        raise RuntimeError(message + '; immediate stop')
    return errors, currents


def _read_sample(bus, command, report, phase, snapshot_check=None):
    getattr(bus,'cancel_callback',check_cancel)()
    before = time.monotonic()
    state = bus.snapshot()
    age = time.monotonic() - before
    errors = np.abs(state['position'] - command) * TICK
    current = signed_current(state['current'])
    report.setdefault('samples', []).append(dict(phase=phase, t=time.monotonic()-report['enabled_at'],
        goal=command.tolist(), goal_readback=state['goal'].tolist(),
        actual=state['position'].tolist(), current_ma=current.tolist(),
        voltage_v=(state['voltage']/10).tolist(), temperature_c=state['temperature'].tolist(),
        tracking_rad=errors.tolist(), watchdog=state['watchdog'].tolist(),
        torque=state['torque'].tolist(), error=state['error'].tolist(), read_ms=age*1000))
    # Optional extended telemetry, decoded from the same packet by diagnostic buses.
    row = report['samples'][-1]
    for key, bits in (('present_pwm',16), ('present_velocity',32)):
        if key in state:
            raw = np.asarray(state[key],dtype=np.int64)
            row[key+'_raw'] = np.where(raw >= 1 << (bits-1),raw-(1 << bits),raw).tolist()
    if 'goal_pwm' in state:
        row['goal_pwm_raw'] = np.asarray(state['goal_pwm']).tolist()
    if age > .2:
        raise RuntimeError(f'Motor read took {age*1000:.0f}ms; no target advance')
    if snapshot_check is not None:
        snapshot_check(state)
    return state, errors


def _sample_healthy(bus, command, report, phase, *, snapshot_check=None):
    state, errors = _read_sample(bus, command, report, phase, snapshot_check)
    profile = getattr(bus,'checked_control_profile',{})
    try:
        health(state, command, **profile)
        return state, errors
    except GoalReadbackMismatch:
        if not getattr(bus, 'goal_readback_confirmation', False):
            raise
        from goal_readback_confirmation import confirm_previous_goal
        return confirm_previous_goal(bus, command, state, report, phase,
            read_sample=lambda: _read_sample(bus, command, report, phase, snapshot_check),
            validate=lambda fresh: health(fresh, command, **profile))
    except MildUndervoltage:
        pass
    began = time.monotonic()
    event = dict(phase=phase, t=began-report['enabled_at'], status='confirming',
                 initial_voltage_v=(state['voltage']/10).tolist(), confirmation_reads=0,
                 goals_advanced=0)
    events = report.setdefault('voltage_events', [])
    events.append(event)
    try:
        recent = sum(event['t']-e['t'] <= VOLTAGE_REPEAT_WINDOW_S for e in events)
        if recent >= VOLTAGE_REPEAT_TRIP:
            raise RuntimeError('Repeated undervoltage: 3 events within 2s; torque OFF')
        print('Voltage dip: holding last goal; fresh-read confirmation budget 150ms.', flush=True)
        good = 0
        for _ in range(VOLTAGE_CONFIRM_MAX_READS):
            getattr(bus,'cancel_callback',check_cancel)()
            if time.monotonic()-began >= VOLTAGE_CONFIRM_S:
                break
            # No goal, current, mode, or torque write here; watchdog still monitored.
            state, errors = _read_sample(bus, command, report, phase, snapshot_check)
            event['confirmation_reads'] += 1
            event['last_voltage_v'] = (state['voltage']/10).tolist()
            try:
                health(state, command, **profile)
            except MildUndervoltage:
                good = 0
            else:
                good = good + 1 if np.all(state['voltage'] >= VOLTAGE_RECOVER_TENTHS) else 0
            elapsed = time.monotonic()-began
            if elapsed >= VOLTAGE_CONFIRM_S:
                break
            if good >= VOLTAGE_RECOVER_READS:
                event.update(status='recovered', elapsed_ms=elapsed*1000)
                print('Voltage recovered: all IDs >=3.8V twice; continuing from the same step.', flush=True)
                return state, errors
            time.sleep(.01)
        raise RuntimeError('Undervoltage not recovered to >=3.8V twice within 150ms/read limit; torque OFF')
    except (Exception, KeyboardInterrupt) as exc:
        event.update(status='stopped', elapsed_ms=(time.monotonic()-began)*1000,
                     error=str(exc) or 'Interrupted')
        raise


def sample(bus, command, report, phase, *, snapshot_check=None):
    began=time.monotonic()
    failures=len(report.get('read_failures',[]))
    state,errors=_sample_healthy(bus,command,report,phase,snapshot_check=snapshot_check)
    plan=getattr(bus,'plan_module',None)
    if (getattr(plan,'CONFIRM_FRESH_AFTER_READ_RECOVERY',False)
            and len(report.get('read_failures',[]))>failures):
        from read_recovery_confirmation import confirm_recovered_read
        profile=getattr(bus,'checked_control_profile',{})
        return confirm_recovered_read(bus,command,state,errors,report,phase,began=began,
            read_sample=lambda:_read_sample(bus,command,report,phase,snapshot_check),
            validate=lambda fresh:health(fresh,command,**profile),now=time.monotonic)
    return state,errors


def move(bus, command, target, report, phase):
    report['phase'] = phase
    began = time.monotonic()
    state, _ = sample(bus, command, report, phase)
    start = state['position'].copy()  # Original script rereads actual position at each stage.
    if np.max(np.abs(start-command))*TICK > MAX_TRACKING_RAD:
        raise RuntimeError(f'{phase}: prior stage not close enough to start')
    for step in range(1, STEPS+1):
        check_cancel()
        if time.monotonic()-began > MAX_PHASE_S:
            raise RuntimeError(f'{phase}: exceeded {MAX_PHASE_S}s')
        next_command = validate_raw(np.rint(start+(target-start)*(step/STEPS)))
        bus.write(116,4,next_command,verify=False)
        command = next_command
        time.sleep(DELAY_S)
        if step % SAMPLE_EVERY_STEPS == 0 or step == STEPS:
            state, errors = sample(bus, command, report, phase)
            worst = int(np.argmax(errors))
            report.setdefault('stage_diagnostics', []).append(dict(phase=phase, step=step,
                max_tracking_rad=float(errors[worst]), worst_id=worst+1,
                current_ma=int(signed_current(state['current'])[worst])))
            if step % 20 == 0 or step == STEPS:
                print(f'{phase}: step {step}/{STEPS}; ID{worst+1} error {errors[worst]:.3f}rad '
                      f'current {signed_current(state["current"])[worst]}mA',flush=True)
    time.sleep(.3)  # Original pause between stages; no forced settling to a different rhythm.
    state, errors = sample(bus, command, report, phase)
    lagging = np.flatnonzero(errors > .07)
    report.setdefault('stage_end_tracking', {})[phase] = dict(
        max_error_rad=float(np.max(errors)),
        lagging_ids=[int(i)+1 for i in lagging],
        goal=command.tolist(), actual=state['position'].tolist())
    for i in lagging:
        print(f'{phase}: ID{i+1} ended {errors[i]:.3f}rad from goal '
              f'(goal {command[i]}, actual {state["position"][i]}); commands continued.',flush=True)
    return command


class SupportPhotoPreview:
    """Show the actual PLACE frame while the motor loop keeps doing health reads."""

    def __init__(self, recorder):
        self.recorder = recorder
        self.cv = recorder.camera.cv
        self.title = 'LEAP SUPPORT PHOTO - check cube bottom and support'
        self.created = False
        self.last_shown = 0.0

    def show(self):
        if time.monotonic() - self.last_shown < .35:
            return self._key() if self.created else False
        with self.recorder.frame_lock:
            latest = self.recorder.last_analyzed
            if latest is None:
                return False
            stamp, frame, row = latest
            frame = frame.copy()
        if time.monotonic() - stamp > .5:
            return False
        if not self.created:
            self.cv.namedWindow(self.title, self.cv.WINDOW_NORMAL)
            self.cv.resizeWindow(self.title, 1100, 620)
            self.created = True
        height, width = frame.shape[:2]
        margin = max(50, int(height * .09))
        self.cv.line(frame, (0, margin), (width - 1, margin), (0, 255, 255), 2)
        self.cv.line(frame, (0, height - margin), (width - 1, height - margin), (0, 255, 255), 2)
        ids = [i for i in row.get('detected_ids', []) if 0 <= i < 6]
        lines = ('PLACE: show WHOLE cube between yellow lines',
                 'Cube BOTTOM EDGE and supporting finger must BOTH be visible',
                 f'Cube marker IDs {ids}; SECOND SPACE here or in TERMINAL saves; Q stops')
        for index, line in enumerate(lines):
            self.cv.putText(frame, line, (10, 25 + 25 * index),
                            self.cv.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 2)
        self.cv.imshow(self.title, frame)
        self.last_shown = time.monotonic()
        return self._key()

    def _key(self):
        key = self.cv.waitKey(1) & 255
        if key in (ord('q'), 27):
            raise KeyboardInterrupt('User stopped support-photo preview')
        return key == 32

    def close(self):
        if self.created:
            self.cv.destroyWindow(self.title)


def wait_for_cube(bus, command, report, recorder=None, *, capture_only=False,
                  anchor_capture=False, support_photo=False):
    import msvcrt
    report['phase'] = 'place_cube'
    two_face = bool(anchor_capture and recorder is not None
                    and recorder.anchor_min_cube_markers == 2)
    if capture_only:
        if support_photo:
            print('PLACE CUBE on the first pose. Show cube underside and actual support in camera.', flush=True)
            print('After placement, remove your hand from the fingers before SECOND SPACE.', flush=True)
        elif anchor_capture:
            print('PLACE CUBE on the first pose. Show mount ID41 plus '
                  + ('TWO cube markers' if two_face else 'at least ONE cube marker')
                  + '; ID40 may be hidden.', flush=True)
        else:
            print('PLACE CUBE on the first pose. Show ID40 plus TWO cube markers in the camera.', flush=True)
        print(('SECOND SPACE in camera window OR terminal' if support_photo else 'SPACE in this terminal')
              + ' captures one frame, then turns torque OFF. Q stops.', flush=True)
    else:
        print('PLACE CUBE on the recorded first pose; SPACE in this terminal starts one sequence pass. Q stops.', flush=True)
    print('Reference successful start: GREEN face UP, orange-colored face toward the camera.',flush=True)
    if capture_only:
        if support_photo:
            print('PHOTO ONLY: no marker requirement after movement; no 3D registration or policy.', flush=True)
        else:
            print(('ID40 + ID41 empty, then ID41 + '
                   + ('TWO cube markers placed are REQUIRED.' if two_face
                      else 'a cube marker placed are REQUIRED. One-marker pose is provisional.')
                   if anchor_capture else 'Two cube markers and ID40 in one frame are REQUIRED for this measurement.'), flush=True)
    else:
        print('That capture showed cube marker IDs 5 and 3. The camera check below is a hint, not a motion gate.',flush=True)
    last = 0
    capture_started = time.monotonic()
    live_preview = SupportPhotoPreview(recorder) if support_photo and recorder is not None else None
    try:
        while True:
            if capture_only and time.monotonic() - capture_started > 90:
                raise RuntimeError('Placement capture exceeded 90s; torque OFF now')
            state, _ = sample(bus, command, report, 'place_cube')
            camera_space = live_preview.show() if live_preview is not None else False
            if time.monotonic()-last > 2:
                rows = recorder.report['frames'] if recorder is not None else []
                visible = rows[-1]['detected_ids'] if rows else []
                if anchor_capture and rows:
                    visible = sorted(set(visible) | set(rows[-1].get('anchor_detected_ids', [])))
                    print(f'Holding first pose; camera IDs {visible} '
                          f'(ID40 may be hidden; ID41 + {"2 cube markers" if two_face else "cube marker"} needed); '
                          'waiting for SPACE.', flush=True)
                elif support_photo:
                    print('Holding first pose; check LIVE camera window: whole cube, bottom edge, support. '
                          'SECOND SPACE in camera or terminal saves photo and turns torque OFF.', flush=True)
                else:
                    print(f'Holding first pose; camera IDs {visible}; waiting for SPACE.', flush=True)
                last = time.monotonic()
            if anchor_capture and recorder is not None and recorder.anchor_cube_candidate is not None:
                candidate_stamp, _, candidate_row = recorder.anchor_cube_candidate
                if time.monotonic() - candidate_stamp < 3 and time.monotonic() - report.get('_anchor_ready_printed_at', 0) > 1:
                    print(f'CAPTURE READY: ID41 + cube {candidate_row["detected_ids"]}; SPACE in terminal now.', flush=True)
                    report['_anchor_ready_printed_at'] = time.monotonic()
            if camera_space or msvcrt.kbhit():
                key = ' ' if camera_space else msvcrt.getwch().lower()
                if key in ('q', '\x1b'):
                    raise KeyboardInterrupt('User stop')
                if key == ' ':
                    if recorder is not None:
                        try:
                            row = recorder.mark_placement(require_two_cube=capture_only and not anchor_capture and not support_photo,
                                                          require_anchor_cube=anchor_capture,
                                                          require_support_photo=support_photo)
                        except RuntimeError as exc:
                            print(f'{exc}. Adjust camera/cube; Q stops.', flush=True)
                            continue
                    else:
                        row = None
                    if row:
                        report['cube_placement_camera_snapshot'] = dict(
                            detected_ids=row['detected_ids'],reason=row['reason'],
                            captured_at_unix_s=row['captured_at_unix_s'],
                            capture_age_s=row.get('capture_age_s', 0.0),
                            stale=row.get('capture_stale', False))
                        if row.get('capture_stale'):
                            print(f'Last ID41 + cube frame was {row["capture_age_s"]:.1f}s old. '
                                  'Saving diagnostic only; torque OFF now.', flush=True)
                    return
            time.sleep(.05)
    finally:
        if live_preview is not None:
            live_preview.close()


def wait_for_removal(bus, command, report):
    import msvcrt
    report['phase'] = 'remove_cube'
    print('REMOVE CUBE while goals hold; SPACE in this terminal confirms removal and turns torque OFF. Q stops.', flush=True)
    while True:
        sample(bus, command, report, 'remove_cube')
        if msvcrt.kbhit():
            key = msvcrt.getwch().lower()
            if key in ('q', '\x1b'):
                raise KeyboardInterrupt('User stop')
            if key == ' ':
                return
        time.sleep(.05)


def run(bus, initial, poses, report, *, empty_check=False, recorder=None,
        capture_only=False, anchor_capture=False, support_photo=False, prepared_callback=None):
    if prepared_callback is not None and not empty_check:
        raise ValueError('Prepared callback requires empty preparation mode')
    profile = validate_control_profile(getattr(bus,'checked_control_profile',{}))
    if profile:bus.checked_control_profile=profile
    caps = np.asarray(profile.get('caps',CAP))
    gains = profile.get('gains',GAINS)
    report['applied_control_profile'] = dict(current_caps_ma=caps.tolist(),
        gains={k:np.asarray(v).tolist() for k,v in gains.items()},
        measured_current_trip_ma=np.broadcast_to(profile.get('current_trip_ma',CURRENT_TRIP_MA),(16,)).tolist())
    original = initial['position'].copy()
    if np.max(np.abs(poses[0][1]-original))*TICK > 2.5:
        raise RuntimeError('First pose exceeds 2.5rad from current position; no movement')
    report['initial'] = {k:v.tolist() for k,v in initial.items()}
    report['targets'] = [p.tolist() for _,p in poses]
    report['stage_names'] = [name for name,_ in poses]
    report['voltage_policy'] = dict(version='fresh_confirm_v1', minimum_v=3.7,
        confirm_only_v=3.6, confirm_seconds=VOLTAGE_CONFIRM_S, recovery_v=3.8,
        recovery_reads=VOLTAGE_RECOVER_READS, max_confirmation_reads=VOLTAGE_CONFIRM_MAX_READS,
        repeated_events_trip=VOLTAGE_REPEAT_TRIP, repeated_window_seconds=VOLTAGE_REPEAT_WINDOW_S,
        enable_checks='strict', goals_advanced_during_confirmation=0)
    print('Voltage check v1: 3.6V -> hold goals + fresh reads (150ms budget); '
          '2 x all IDs >=3.8V resumes; <=3.5V stops.', flush=True)
    touched = False
    try:
        touched = True
        report['phase'] = 'configure_off'
        for addr, key in ((80,'d_gain'),(82,'i_gain'),(84,'p_gain')):
            bus.write(addr, 2, gains[key])
        bus.write(102, 2, caps)
        bus.write(108, 4, np.zeros(16,dtype=int))
        bus.write(112, 4, np.zeros(16,dtype=int))
        bus.write(116, 4, original, verify=False)
        for _ in range(2):
            state = bus.snapshot()
            if (np.any(state['torque']) or np.any(state['error']) or np.any(state['watchdog'])
                    or np.max(np.abs(state['goal']-original)) > 4
                    or np.max(np.abs(state['position']-original)) > 4
                    or np.max(np.abs(state['goal']-state['position'])) > 4):
                raise RuntimeError('OFF preload did not hold near current angles; no enable')
            time.sleep(.02)
        bus.write(98, 1, np.full(16,25))
        check_cancel()
        report['phase'] = 'enable_hold'
        bus.write(64, 1, np.ones(16,dtype=int))
        report['enabled_at'] = time.monotonic()
        first_goal = None
        for _ in range(2):
            check_cancel()
            state = bus.snapshot()
            command = state['goal'].copy()
            report.setdefault('enable_snapshots', []).append({k:v.tolist() for k,v in state.items()})
            health(state, command, **profile)
            if (np.max(np.abs(state['position']-original)) > 4 or
                    np.max(np.abs(command-original)) > 4 or
                    np.max(np.abs(state['position']-command)) > 4):
                raise RuntimeError('Enable shifted hand/goal more than 4 ticks')
            if first_goal is not None and np.max(np.abs(command-first_goal)) > 1:
                raise RuntimeError('Enable hold goal not stable')
            first_goal = command.copy()
            time.sleep(.02)
        bus.write(116, 4, command)
        for i,(name,target) in enumerate(poses):
            if i == 1 and not empty_check:
                if anchor_capture:
                    report['anchor_rigid_projection_error_px'] = recorder.check_anchor_rigidity()
                wait_for_cube(bus, command, report, recorder, capture_only=capture_only,
                              anchor_capture=anchor_capture, support_photo=support_photo)
                if capture_only:
                    stale = report.get('cube_placement_camera_snapshot', {}).get('stale', False)
                    report['status'] = ('support_photo_complete' if support_photo else
                                        'placement_capture_stale' if stale else 'placement_capture_complete')
                    print(('Support photo saved (qualitative only).' if support_photo else
                           'Stale placement image saved for diagnosis.' if stale else 'Placement frame captured.')
                          + ' Turning torque OFF; support/remove cube as the hand relaxes.', flush=True)
                    return
            print(f'Stage {i}/{len(poses)-1}: {name}', flush=True)
            command = move(bus, command, target, report, f'stage_{i}')
        if prepared_callback is not None:
            prepared_callback(bus, command, report)
        if not empty_check:
            wait_for_removal(bus, command, report)
        report['status'] = ('revised_start_capture_complete' if prepared_callback is not None else
                            'empty_check_complete' if empty_check else 'sequence_complete')
        print('One recorded command sequence complete; physical cube flip is not verified. Turning torque OFF.', flush=True)
    finally:
        if touched:
            report['all_torque_off_verified'] = False
            bus.off()
            report['all_torque_off_verified'] = True
            bus.write(98,1,np.zeros(16,dtype=int))
            for addr,size,key in ((102,2,'current_cap'),(108,4,'acceleration'),(112,4,'velocity'),(98,1,'watchdog'),
                                  (80,2,'d_gain'),(82,2,'i_gain'),(84,2,'p_gain')):
                bus.write(addr,size,initial[key])
            report['settings_restored'] = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true', help='parse poses and validate without opening motor port')
    parser.add_argument('--empty-check', action='store_true', help='move once without cube placement, then torque OFF')
    parser.add_argument('--record-vision', action='store_true',
                        help='record the named webcam alongside the unchanged sequence')
    parser.add_argument('--capture-placement-only', action='store_true',
                        help='move only to stage 0, capture ID40 and two cube markers together, then torque OFF')
    parser.add_argument('--capture-anchor-placement', action='store_true',
                        help='capture ID40+41 empty then ID41+one or more cube markers placed; no stages 1-6')
    parser.add_argument('--anchor-two-face', action='store_true',
                        help='with anchor capture, require fresh ID41 plus two cube markers')
    parser.add_argument('--capture-support-photo', action='store_true',
                        help='stage 0 and one fresh qualitative support photo; no marker gate after movement')
    args = parser.parse_args()
    if args.capture_support_photo and args.capture_anchor_placement:
        parser.error('Choose either support photo or anchor placement')
    if args.anchor_two_face and not args.capture_anchor_placement:
        parser.error('--anchor-two-face requires --capture-anchor-placement')
    if args.capture_anchor_placement:
        args.capture_placement_only = True
    if args.capture_support_photo:
        args.capture_placement_only = True
    if args.capture_placement_only and (args.empty_check or not args.record_vision):
        parser.error('--capture-placement-only requires --record-vision and excludes --empty-check')
    digest, poses = load_sequence()
    mode = 'stage 0 placement capture only' if args.capture_placement_only else f'{len(poses)} poses; one pass'
    print(f'Local sequence SHA256 {digest}; {mode}.', flush=True)
    for i,(name,pose) in enumerate(poses[:1] if args.capture_placement_only else poses):
        previous = poses[i-1][1] if i else pose
        print(f'Stage {i}: max step {np.max(np.abs(pose-previous))*TICK:.3f}rad; {name}', flush=True)
    if args.check:
        print('CHECK ONLY: no COM port opened, no motor write.', flush=True)
        return 0
    report = dict(timestamp=dt.datetime.now().astimezone().isoformat(), source=str(POSES_FILE),
                  source_sha256=digest, status='not_started', phase='preflight',
                  all_torque_off_verified=False, settings_restored=False, policy_used=False,
                  physical_flip_verified=False,
                  empty_check=args.empty_check, capture_placement_only=args.capture_placement_only,
                  capture_anchor_placement=args.capture_anchor_placement,
                  capture_support_photo=args.capture_support_photo,
                  anchor_two_face=args.anchor_two_face,
                  hardware_authorized=False)
    bus = None
    recorder = None
    opened = False
    try:
        if args.record_vision:
            from sequence_vision_recorder import SequenceVisionRecorder
            recorder = SequenceVisionRecorder(ROOT, report,
                                              anchor_mode=args.capture_anchor_placement,
                                              support_photo_mode=args.capture_support_photo,
                                              anchor_min_cube_markers=2 if args.anchor_two_face else 1)
            print('Camera recording started; no second command needed.',flush=True)
        import dynamixel_sdk as sdk
        bus = Bus(sdk, report)
        if not bus.port.openPort():
            raise RuntimeError('Cannot open COM7')
        opened = True
        if not bus.port.setBaudRate(3000000):
            raise RuntimeError('Cannot set 3000000 baud')
        initial = bus.preflight()
        print('EMPTY hand for initial movement. Cube only after PLACE prompt. Q/Esc/Ctrl+C stops.', flush=True)
        if args.empty_check:
            print('EMPTY-ONLY CHECK: run all seven poses without a cube, then torque OFF.',flush=True)
        if args.capture_support_photo:
            print('SUPPORT PHOTO ONLY: first pose, one frame, OFF. No 3D registration or policy.',flush=True)
            print('FIRST SPACE was for EMPTY-hand Stage 0. Place the cube only when PLACE CUBE appears.', flush=True)
            print('Remove the checkerboard sheet and your hand from all finger paths before countdown.',flush=True)
        print('Use a padded catch area; stop and cut motor power for unexpected contact or noise.', flush=True)
        print('Original dynamics: 450mA, P600/I0/D200 (IDs8/12/16 P450/D150), profile 0.',flush=True)
        print('70 interpolated goals x 0.03s per stage; live health checks; settings restored after OFF.',flush=True)
        print('No automatic motion retry. Stop for overcurrent, low voltage, or excessive lag.',flush=True)
        for n in range(3,0,-1):
            check_cancel()
            print(f'Starting in {n}s; clear fingers/cables.',flush=True)
            time.sleep(1)
        fresh = bus.preflight()
        if np.max(np.abs(fresh['position']-initial['position']))*TICK > .025:
            raise RuntimeError('Hand position changed during countdown; restart')
        run(bus, fresh, poses, report, empty_check=args.empty_check, recorder=recorder,
            capture_only=args.capture_placement_only, anchor_capture=args.capture_anchor_placement,
            support_photo=args.capture_support_photo)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped', error=str(exc) or 'Interrupted', failed_phase=report.get('phase'))
        print('STOP: '+report['error'], flush=True)
    finally:
        if bus and opened:
            try:
                torque = bus.block(64,1,[('torque',64,1)])['torque']
                report['final_torque_states'] = torque.tolist()
                report['all_torque_off_verified'] = bool(np.all(torque == 0))
            except Exception as exc:
                report['final_check_error'] = str(exc)
                report['all_torque_off_verified'] = False
            if not report['all_torque_off_verified']:
                print('CUT MOTOR POWER NOW: torque OFF not verified.', flush=True)
            bus.port.closePort()
        if recorder is not None:
            try:
                report['vision_recording'] = recorder.finish()
                print('Vision saved:',report['vision_recording']['folder'],flush=True)
                print('Vision by phase:',{phase: round(summary['eligible_percent'],1)
                    for phase,summary in report['vision_recording']['by_phase'].items()},flush=True)
                if args.capture_anchor_placement:
                    rows = recorder.report['frames']
                    placed = [row for row in rows if row['phase'] == 'place_cube']
                    if placed:
                        seen40 = sum(40 in row.get('anchor_detected_ids', []) for row in placed)
                        seen41 = sum(41 in row.get('anchor_detected_ids', []) for row in placed)
                        print(f'Placed-cube anchor visibility: ID40 {seen40}/{len(placed)}, '
                              f'ID41 {seen41}/{len(placed)}. ID40 can be occluded by the cube.',
                              flush=True)
            except Exception as exc:
                report['vision_recording'] = dict(status='save_failed',error=str(exc))
                print('Vision recording could not be saved:',exc,flush=True)
        stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        result_name = ('leap_anchor_two_face_result.json' if args.anchor_two_face else
                       'leap_support_capture_result.json' if args.capture_support_photo else
                       'leap_anchor_capture_result.json' if args.capture_anchor_placement else
                       'leap_registration_capture_result.json' if args.capture_placement_only else
                       'leap_sequence_flip_result.json')
        for path in (ROOT/f'leap_sequence_flip_{stamp}.json', ROOT/result_name):
            path.write_text(json.dumps(report,ensure_ascii=False,indent=2,default=float),encoding='utf-8')
        if report.get('samples'):
            rows = report['samples']
            print(f'Measured: min voltage {min(min(row["voltage_v"]) for row in rows):.1f}V; '
                  f'peak current {max(abs(value) for row in rows for value in row["current_ma"])}mA; '
                  f'max tracking {max(max(row["tracking_rad"]) for row in rows):.3f}rad.',flush=True)
        failures = report.get('read_failures', [])
        if failures:
            print(f'Fresh read recovery: {sum(bool(row.get("recovered")) for row in failures)} recovered; '
                  f'{sum(not row.get("recovered") for row in failures)} unrecovered read events.',flush=True)
        anomalies = report.get('preflight_static_read_anomalies', [])
        for row in anomalies:
            print(f'ID{row["motor_id"]} {row["register"]} reads {row["values"]}; '
                  f'{"confirmed transient" if row["recovered"] else "persistent mismatch"}.',flush=True)
        print('Saved:',ROOT/result_name,flush=True)
    return 0 if report['status'] in ('sequence_complete','empty_check_complete',
                                     'placement_capture_complete','support_photo_complete') and report['all_torque_off_verified'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
