"""Operator-started replay of the user's measured physical grip, then hold.

Default/check mode opens no devices. The operator starts a slow joint-pose
reproduction followed by fixed hold. No cube-placement workflow is imposed.
This does not control cube position, transfer or rotation.
"""
from pathlib import Path
import argparse, datetime as dt, hashlib, json, time
from types import SimpleNamespace
import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT/'right_hand_rotation_20261117/user_measured_grip'
REFERENCE = OUT/'reference.json'
MANIFEST = OUT/'pose_runner_manifest.json'
PERIOD = .075
MOVE_PERIOD = .060  # Authorized after the recorded747-goal physical success.
MAX_ON_SECONDS = 500.
FIELDS = [('torque',64,1), ('error',70,1), ('d_gain',80,2), ('i_gain',82,2),
    ('p_gain',84,2), ('watchdog',98,1), ('current_cap',102,2), ('acceleration',108,4),
    ('velocity',112,4), ('goal',116,4), ('current',126,2), ('position',132,4),
    ('voltage',144,2), ('temperature',146,1)]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def reference():
    r = json.loads(REFERENCE.read_bytes())
    source = ROOT/r['source_path']
    if sha(source) != r['source_sha256'] or not r.get('preferred_reference_over_independent_model_grip'):
        raise RuntimeError('Measured reference/source mismatch; no devices opened')
    raw = json.loads(source.read_bytes())
    motors = sorted(raw['snapshots'][-1]['motors'], key=lambda row: row['id'])
    if [row['id'] for row in motors] != list(range(1,17)) or raw['errors']:
        raise RuntimeError('Incomplete measured reference')
    actual = np.asarray(r['measured_ticks'], dtype=int)
    goal = np.asarray(r['holding_goal_ticks_readonly'], dtype=int)
    if (actual.shape != (16,) or goal.shape != (16,)
        or actual.tolist() != [row['position_ticks'] for row in motors]
        or goal.tolist() != [row['goal_ticks_readonly'] for row in motors]):
        raise RuntimeError('Recorded measured/holding goals were edited')
    return r, actual, goal


def bounds():
    from loaded_quarter_reentry_20261025 import limits
    from final_pose_start_scope_20261116 import PASSIVE_LOWER, ROOT_INDICES
    low, high = [v.copy() for v in limits()]
    admission_low = low.copy()
    admission_low[ROOT_INDICES] = PASSIVE_LOWER
    # This runner alone permits the exact physically recorded ID11 target.
    # No original limit file, EEPROM setting or other joint limit is changed.
    high[10] = max(float(high[10]), float(reference()[2][10]))
    return np.ceil(admission_low).astype(int), np.floor(high).astype(int), low


def vector(value):
    a = np.asarray(value)
    if a.shape != (16,) or not np.isfinite(a).all() or np.any(a != np.rint(a)):
        raise ValueError('Expected16 integer motor ticks')
    return a.astype(int)


def encoder_vector(value):
    a=vector(value)
    if np.any(a<0) or np.any(a>4095):
        raise ValueError('Invalid single-turn encoder value; no wrapping or motor writes')
    return a


def outside_excess(value):
    _,high,low=bounds()
    return np.maximum(np.maximum(low-value,value-high),0.)


def validate_goal_return(value,previous,entry):
    a=encoder_vector(value); old=encoder_vector(previous); anchor=encoder_vector(entry)
    _,high,low=bounds()
    if np.any(a<np.minimum(np.ceil(low),anchor)) or np.any(a>np.maximum(high,anchor)):
        raise RuntimeError('Goal goes beyond the measured start/target envelope')
    if np.max(abs(a-old))>2:
        raise RuntimeError('Goal step exceeds2ticks')
    if np.any(outside_excess(a)>outside_excess(old)+1e-9):
        raise RuntimeError('Outside-range joint may only return toward its target range')
    return a


def path(start, target):
    start, target = encoder_vector(start), encoder_vector(target)
    _, high, native_low = bounds()
    if np.any(target < native_low) or np.any(target > high):
        raise RuntimeError('Measured target outside its scoped pose limits')
    # First return only out-of-range joints. The remaining joints keep their
    # exact current goals until that return completes. No outward extension.
    joined=np.clip(start,np.ceil(native_low),high).astype(int)
    pieces=[]; previous=start
    for endpoint in (joined,target):
        n=int(np.ceil(np.max(abs(endpoint-previous))/2))
        if n:
            pieces.append(np.rint(previous+(endpoint-previous)*np.arange(1,n+1)[:,None]/n).astype(int))
        previous=endpoint
    rows=np.vstack(pieces) if pieces else target[None,:].copy()
    if len(rows)*PERIOD+10 >= MAX_ON_SECONDS:
        raise RuntimeError('Pose movement exceeds existing500second budget')
    whole=np.vstack([start,rows])
    if np.max(abs(np.diff(whole,axis=0))) > 2:
        raise RuntimeError('Two-tick goal bound')
    excess=np.maximum(np.maximum(native_low-whole,whole-high),0.)
    if (np.any(np.diff(excess,axis=0)>1e-9)
        or np.any(rows<np.minimum(start,np.ceil(native_low)))
        or np.any(rows>np.maximum(start,high)) or not np.array_equal(rows[-1],target)):
        raise RuntimeError('Pose path range/endpoint mismatch')
    return rows


def validate_sources():
    m = json.loads(MANIFEST.read_bytes())
    if m.get('kind') != 'user_measured_grip_pose_reproduction':
        raise RuntimeError('Wrong pose-runner manifest')
    changed = [name for name, digest in m['source_hashes'].items() if sha(ROOT/name) != digest]
    if changed:
        raise RuntimeError('Runner sources changed: '+', '.join(changed))
    reference()
    return m


class OperatorStop(Exception):
    pass


class Terminal:
    def wait(self):
        import msvcrt
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getwch()
                if key == ' ': return
                if key in ('q','Q','\x1b','\x03'): raise OperatorStop()
            time.sleep(.02)

    def check(self):
        import msvcrt
        while msvcrt.kbhit():
            if msvcrt.getwch() in (' ', 'q', 'Q', '\x1b', '\x03'):
                raise OperatorStop()


class PoseBus:
    """Narrow RAM write allowlist, paired ordinary full-health reads."""
    def __init__(self, report, cancel):
        import dynamixel_sdk as sdk
        from serial.tools.list_ports import comports
        devices = [p for p in comports() if p.vid == 0x0403 and p.pid == 0x6014
                   and p.serial_number == 'FTBENPSAA']
        if len(devices) != 1:
            raise RuntimeError('Known robot USB adapter missing or ambiguous')
        self.sdk, self.report, self.cancel_callback = sdk, report, cancel
        self.port = sdk.PortHandler(devices[0].device)
        self.packet = sdk.PacketHandler(2.)
        if not self.port.openPort(): raise RuntimeError('Cannot open robot serial port')
        if not self.port.setBaudRate(1000000):
            self.port.closePort()
            raise RuntimeError('Cannot set host1Mbps; no servo settings changed')
        report['port'] = devices[0].device
        try:
            from calibrated_precise_clock_20261028 import install as install_clock
            install_clock(self)
            from grip_health_read_20261118 import install as install_reader
            install_reader(self)
        except BaseException:
            self.port.closePort()
            raise
        self.plan_module = SimpleNamespace(NEW_PAIR_GRASP_MODE=True)
        self.goal_readback_confirmation = False
        self.previous_goal_command = self.last_goal_command = None
        self.entry_anchor = None
        self.controlled = False

    def read(self, motor, address, size=1):
        fn = {1:self.packet.read1ByteTxRx, 2:self.packet.read2ByteTxRx, 4:self.packet.read4ByteTxRx}[size]
        value, comm, error = fn(self.port, motor, address)
        if comm or error: raise RuntimeError(f'ID{motor} read{address}: comm={comm}, error={error}')
        return value

    def snapshot(self):
        from grip_health_read_20261118 import block_once
        began = time.perf_counter()
        previous_deadline = getattr(self, '_horizontal_sample_deadline', None)
        self._horizontal_sample_deadline = min(began+.2, previous_deadline if previous_deadline is not None else float('inf'))
        failures, clean = [], 0
        try:
            for attempt in range(5):
                if time.perf_counter() >= self._horizontal_sample_deadline: break
                state, comm, error, details = block_once(self, 64, 83, FIELDS)
                if not comm and not error and state is not None:
                    if time.perf_counter() >= self._horizontal_sample_deadline: break
                    if failures:
                        clean += 1
                        if clean < 2: continue
                        for event in failures:
                            event.update(recovered=True,consecutive_clean_reads=clean,
                                         total_elapsed_ms=1000*(time.perf_counter()-began))
                    return state
                event = dict(address=64, communication=int(comm), device_error=int(error),
                             recovered=False, details=details, time=time.monotonic(),
                             attempt=attempt+1,elapsed_ms=1000*(time.perf_counter()-began))
                self.report.setdefault('read_failures', []).append(event)
                failures.append(event);clean=0
                # Existing READ policy permits up to two retransmissions.
                # A second bad packet must not bypass the remaining shared
                # deadline/rate budget. No partial state or motion is accepted.
                if error or len(failures)>=3 or comm not in (-3001,-3002):
                    raise RuntimeError('Repeated/error health read; no target advance')
                stamp = time.perf_counter()
                recent = [t for t in getattr(self,'_read_recovery_times',[]) if stamp-t<10.]
                if len(recent)>=3: raise RuntimeError('Existing3retries/10s limit reached; no target advance')
                from combined_quiet_reserve_20261029 import quiet, POLICY
                event['quiet_resynchronization'] = {}
                event['recovery_policy'] = POLICY['version']
                quiet(self,self._horizontal_sample_deadline,event['quiet_resynchronization'])
                self._read_recovery_times = recent+[time.perf_counter()]
            raise RuntimeError('Full16 health read not confirmed within200ms; no target advance')
        finally:
            if previous_deadline is None: del self._horizontal_sample_deadline
            else: self._horizontal_sample_deadline = previous_deadline

    def write(self, address, size, values):
        allowed = {64:1, 98:1, 102:2, 108:4, 112:4, 116:4}
        if allowed.get(address) != size:
            raise RuntimeError('Write outside pose-runner RAM allowlist')
        a = vector(values)
        if address == 116:
            if self.entry_anchor is None or self.last_goal_command is None:
                raise RuntimeError('No measured starting goal bound')
            a=validate_goal_return(a,self.last_goal_command,self.entry_anchor)
        writer = self.sdk.GroupSyncWrite(self.port, self.packet, address, size)
        try:
            for i, value in enumerate(a, 1):
                if not writer.addParam(i, list(int(value).to_bytes(size, 'little', signed=False))):
                    raise RuntimeError('Cannot assemble complete16 motor packet')
            if writer.txPacket() != self.sdk.COMM_SUCCESS:
                raise RuntimeError(f'Write{address} failed; no write retry')
        finally:
            writer.clearParam()
        self.report.setdefault('writes', []).append(dict(address=address, values=a.tolist()))
        if address == 116:
            self.previous_goal_command = self.last_goal_command
            self.last_goal_command = a.copy()

    def close(self):
        self.port.closePort()

    def off(self):
        try:
            self.write(64, 1, np.zeros(16,dtype=int))
            off = self.snapshot()
            if np.any(off['torque']): raise RuntimeError('Broadcast OFF not confirmed for all16 motors')
        except Exception as exc:
            self.report['broadcast_off_error'] = str(exc)
            # Emergency OFF may fall back to individually addressed commands;
            # position/settings writes are never retried.
            failures = []
            for motor in range(1,17):
                try:
                    comm, error = self.packet.write1ByteTxRx(self.port, motor, 64, 0)
                    if comm or error: failures.append(motor)
                except Exception:
                    failures.append(motor)
            self.report['individual_off_unconfirmed_ids'] = failures
            off = self.snapshot()
        if np.any(off['torque']): raise RuntimeError('Not all16 motors report OFF')
        self.report['all_torque_off_verified'] = True


def preflight(bus, report, target):
    for motor in range(1,17):
        model, comm, error = bus.packet.ping(bus.port, motor)
        if comm or error or model != 1240:
            raise RuntimeError(f'ID{motor} PING/model mismatch; no writes')
        if bus.read(motor, 10) != 0 or bus.read(motor, 11) != 5 or bus.read(motor, 20, 4) != 0:
            raise RuntimeError(f'ID{motor} mode/direction/offset mismatch; no writes')
        if bus.read(motor,6)<38 or bus.read(motor,8)!=3:
            raise RuntimeError(f'ID{motor} firmware/1Mbps register mismatch; no writes')
        if bus.read(motor, 38, 2) < 450:
            raise RuntimeError(f'ID{motor} EEPROM current limit below450mA; no writes')
    s = bus.snapshot()
    report['initial'] = {k:v.tolist() for k,v in s.items()}
    if np.any(s['error']) or np.any(~np.isin(s['watchdog'], [0,25])):
        raise RuntimeError('Motor error or expired watchdog; no writes')
    if not (np.all(s['torque']==0) or np.all(s['torque']==1)):
        raise RuntimeError('Mixed torque state; no writes')
    if np.any(s['voltage']<37) or np.any(s['voltage']>65) or np.any(s['temperature']>45):
        raise RuntimeError('Initial voltage/temperature outside original bounds; no writes')
    if np.any(s['p_gain']<=0): raise RuntimeError('Invalid current PID; no writes')
    encoder_vector(s['position'])
    start = encoder_vector(s['goal'] if np.all(s['torque']) else s['position'])
    rows = path(start, target)
    bus.entry_anchor=start.copy()
    report['start_angle_policy']='valid16 encoder values; outside-range return only; final target unchanged'
    report['initial_outside_ticks']=outside_excess(start).tolist()
    report['start_mode'] = 'adopt_existing_ON_goal' if np.all(s['torque']) else 'preload_OFF_then_enable'
    report['planned_goals'] = len(rows)
    return s, start, rows


def off_health(state, caps, gains):
    if np.any(state['torque']) or np.any(state['error']) or np.any(~np.isin(state['watchdog'],[0,25])):
        raise RuntimeError('OFF motor/error/watchdog state changed; no enable')
    if np.any(state['current_cap']!=caps) or np.any(state['acceleration']) or np.any(state['velocity']):
        raise RuntimeError('OFF current/profile settings changed; no enable')
    if any(not np.array_equal(state[k],v) for k,v in gains.items()):
        raise RuntimeError('OFF PID changed; no enable')
    if np.any(state['voltage']<37) or np.any(state['voltage']>65) or np.any(state['temperature']>45):
        raise RuntimeError('OFF voltage/temperature out of range; no enable')


def settle_off_preload(bus, anchor, caps, gains, report, cancel, *, now, sleep):
    from horizontal_preload_confirmation_20261027 import POLICY
    began=now(); recent=[]
    log=dict(POLICY,accepted=False,goal_position_ticks=1)
    report['off_preload_confirmation']=log
    for _ in range(POLICY['maximum_reads']):
        cancel()
        if now()-began>=POLICY['budget_seconds']:break
        state=bus.snapshot()
        report.setdefault('off_preload_snapshots',[]).append({k:v.tolist() for k,v in state.items()})
        off_health(state,caps,gains)
        comparisons=[('position-request',state['position']-anchor,4),
                     ('goal-request',state['goal']-anchor,4),
                     ('goal-position',state['goal']-state['position'],1)]
        for name,delta,bound in comparisons:
            bad=np.flatnonzero(abs(delta)>bound)
            if len(bad):
                i=int(bad[0]);log['failure']=dict(id=i+1,comparison=name,limit=bound,
                    requested=int(anchor[i]),goal=int(state['goal'][i]),actual=int(state['position'][i]))
                raise RuntimeError(f'ID{i+1} OFF preload {name}: request={anchor[i]}, goal={state["goal"][i]}, actual={state["position"][i]}; limit={bound}ticks; no enable')
        recent.append(state);recent=recent[-POLICY['fresh_reads_required']:]
        if len(recent)==POLICY['fresh_reads_required'] and now()-began<POLICY['budget_seconds']:
            spans={k:np.ptp(np.asarray([s[k] for s in recent]),axis=0) for k in ('position','goal')}
            if all(np.max(v)<=2 for v in spans.values()):
                log.update(accepted=True,elapsed_seconds=now()-began,
                    accepted_hold_goal=state['goal'].tolist(),shift_from_fixed_anchor=(state['goal']-anchor).tolist())
                return state['goal'].copy()
        sleep(.02)
    raise RuntimeError('OFF preload did not settle across3 fresh reads within250ms; no enable')


def run_pose(bus, initial, command, rows, report, cancel, *, now=time.monotonic, sleep=time.sleep, limit=MAX_ON_SECONDS, sequence=None):
    import sequence_flip_checked as checked
    from calibrated_two_full_health_20261028 import sample
    from held_joint_guard import HeldJointGuard
    from goal_aware_progress_guard_20261025 import GoalAwareProgress
    saved = {key:initial[key].copy() for key in ('current_cap','acceleration','velocity','watchdog')}
    caps = np.full(16, 450, dtype=int)
    gains = {key:initial[key].copy() for key in ('p_gain','i_gain','d_gain')}
    bus.checked_control_profile = dict(caps=caps, gains=gains)
    report['enabled_at'] = now()
    bus.last_goal_command = command.copy()
    bus.entry_anchor=command.copy()
    final_target=rows[-1].copy()
    def rebuild(start):
        result=path(start,final_target) if sequence is None else sequence.rows(start)
        if not np.array_equal(result[-1],final_target):
            raise RuntimeError('Sequence endpoint changed during initialization')
        return result
    try:
        # From this point every failure, including partial configuration, goes
        # through OFF verification. Never turn an already-held cube OFF first.
        bus.controlled = True
        bus.write(102, 2, caps)
        bus.write(108, 4, np.zeros(16, dtype=int))
        bus.write(112, 4, np.zeros(16, dtype=int))
        if not np.any(initial['torque']):
            # Capture the actual OFF rest position AFTER configuration, before
            # the first goal write. Never chase a moving position by rewriting.
            fresh=bus.snapshot()
            report['fresh_off_start']={k:v.tolist() for k,v in fresh.items()}
            off_health(fresh,caps,gains)
            command=encoder_vector(fresh['position'])
            rows=rebuild(command)
            bus.last_goal_command=command.copy()
            bus.entry_anchor=command.copy()
            anchor=command.copy()
            bus.write(116, 4, command)
            command=settle_off_preload(bus,anchor,caps,gains,report,cancel,now=now,sleep=sleep)
            bus.last_goal_command=command.copy()
            rows=rebuild(command)
        bus.write(98, 1, np.full(16,25))
        if not np.any(initial['torque']): bus.write(64, 1, np.ones(16,dtype=int))
        report['phase'] = 'initial_hold'
        if not np.any(initial['torque']):
            state=bus.snapshot()
            report.setdefault('enable_snapshots',[]).append({k:v.tolist() for k,v in state.items()})
            checked.health(state,state['goal'],**bus.checked_control_profile)
            if any(np.max(abs(state[k]-anchor))>4 or np.max(abs(state[k]-command))>4 for k in ('goal','position')) or np.max(abs(state['goal']-state['position']))>1:
                raise RuntimeError('Enable hold shifted beyond existing4tick anchors; no target advance')
            command=vector(state['goal'])
            bus.last_goal_command=command.copy()
            rows=rebuild(command)
        else:
            state, _ = sample(bus, command, report, report['phase'])
        previous=state['position'].copy()
        state, _ = sample(bus, command, report, report['phase'])
        report.setdefault('enable_snapshots',[]).append({k:v.tolist() for k,v in state.items()})
        if not np.any(initial['torque']) and (np.max(abs(state['position']-anchor))>4 or np.max(abs(state['position']-command))>4 or np.max(abs(state['position']-previous))>1):
            raise RuntimeError('Two fresh enable snapshots did not remain stable')
        report['accepted_start_goal']=command.tolist()
        # Bind the accepted ON hold once. Initialization can shift by at most
        # the unchanged4tick anchors; every subsequent goal must retreat only.
        bus.entry_anchor=command.copy()
        report['planned_goals']=len(rows)
        if sequence is not None and hasattr(sequence,'metadata'):
            report.update(sequence.metadata())
        guard = HeldJointGuard(state['position'], command)
        progress = GoalAwareProgress()
        bus.goal_readback_confirmation = True
        index, pause_since = 0, None
        print('현재 목표에서 천천히 측정 파지로 이동합니다. SPACE/Q/Esc: 종료 및 OFF.', flush=True)
        while True:
            cycle = now()
            pacing_began = time.perf_counter()
            cancel()
            if cycle-report['enabled_at'] >= limit:
                report['status'] = 'existing500second_limit_reached'
                break
            moving = index < len(rows)
            phase = ('measured_grip_move' if moving else 'measured_grip_hold') if sequence is None else sequence.phase(index,moving)
            cycle_period=(MOVE_PERIOD if moving else PERIOD) if sequence is None else sequence.period(index,moving)
            if sequence is not None and phase!=report.get('sequence_phase'):
                report['sequence_phase']=phase
                print(sequence.message(phase),flush=True)
            report['phase'] = phase
            if not moving and not report.get('holding'):
                report['holding'] = True
                print('파지 자세 도착. 같은 목표를 계속 유지합니다. 받친 뒤 SPACE로 종료/OFF.' if sequence is None else
                      '설정한 경로 끝. 현재 목표를 유지합니다. 큐브를 받친 뒤 SPACE로 종료/OFF.',flush=True)
            target = rows[index] if moving else command
            frozen = np.flatnonzero(target==command) if moving else np.arange(16)
            state, errors = sample(bus, command, report, phase,
                snapshot_check=lambda fresh:guard.check(fresh['position'],command,frozen,phase))
            ready=(sequence is None or not hasattr(sequence,'ready') or sequence.ready(index)) if moving else False
            if moving and ready:
                progress.check(phase, np.flatnonzero(rows[-1]!=command), command, state['position'], now())
                if np.max(abs(target-state['position']))*checked.TICK > .12:
                    pause_since = cycle if pause_since is None else pause_since
                    if cycle-pause_since >= 2:
                        raise RuntimeError('Tracking did not recover within2s; no target advance')
                else:
                    pause_since = None
                    cancel()
                    bus.write(116, 4, target)
                    guard.advance(state['position'], command, target)
                    command = target.copy()
                    index += 1
                    report['last_issued_command'] = command.tolist()
                    report['last_issued_index'] = index-1
                    report['completed_goals'] = index
                    if index % 40 == 0:
                        print(f'자세 이동 {index}/{len(rows)}; 추종 {max(errors):.3f}rad', flush=True)
            delay = cycle_period-(time.perf_counter()-pacing_began)
            if delay > 0: sleep(delay)
    finally:
        # Cleanup does not poll the keyboard; cancellation must not prevent OFF.
        bus.cancel_callback = lambda:None
        try:
            bus.off()
            bus.write(98, 1, np.zeros(16,dtype=int))
            for address, size, key in ((102,2,'current_cap'), (108,4,'acceleration'), (112,4,'velocity'), (98,1,'watchdog')):
                bus.write(address, size, saved[key])
            restored = bus.snapshot()
            if any(not np.array_equal(restored[key], value) for key,value in saved.items()):
                raise RuntimeError('OFF confirmed; RAM restoration incomplete')
            report['settings_restored'] = True
        except BaseException as exc:
            report['cleanup_error'] = str(exc)
            print('종료 확인 실패: 모터 전원을 끄세요. '+str(exc), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute', action='store_true')
    args = ap.parse_args()
    try:
        validate_sources()
        _, measured, target = reference()
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('NOT READY:',exc,'. No devices opened.',flush=True)
        return 2
    if not args.execute:
        print(json.dumps(dict(status='device_free_check_passed', measured_ticks=measured.tolist(),
            target_ticks=target.tolist(), step_ticks=2, period_s=MOVE_PERIOD, hold_period_s=PERIOD, devices_opened=False,
            target_is_user_physical_reference=True, whole_cube_transfer_verified=False), ensure_ascii=False))
        return 0
    from final_grip_single_instance_20261116 import SingleInstance
    terminal = Terminal()
    report = dict(status='started', holding=False, settings_restored=False, all_torque_off_verified=False,
                  reference_sha256=sha(REFERENCE), target_ticks=target.tolist(), rotation_included=False,
                  automatic_open=False, hardware_execution_by_operator=True,
                  move_period_s=MOVE_PERIOD,hold_period_s=PERIOD)
    folder = ROOT/('measured_grip_hold_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    bus = None
    live = None
    try:
        with SingleInstance(ROOT/'.final_grip_pose.lock'):
            print('방금 기록한 실물 파지 자세 재현 → 계속 유지. 회전은 하지 않습니다.', flush=True)
            print('현재 자세에서 기록한 파지 자세로 천천히 이동해 유지합니다.', flush=True)
            print('손을 떼고 이 실행창에서 SPACE를 누르면 시작합니다. Q/Esc: 시작 취소.', flush=True)
            terminal.wait()
            folder.mkdir()
            from measured_grip_live_state_20261117 import LiveState
            live = LiveState(folder, report)
            bus = PoseBus(report, terminal.check)
            initial, start, rows = preflight(bus, report, target)
            report['near_recorded_physical_grip'] = bool(np.max(abs(initial['position']-measured))<=64)
            report['workflow']='current_pose_to_recorded_grip_then_hold'
            run_pose(bus, initial, start, rows, report, terminal.check)
            if report.get('holding') and report['status']=='started': report['status'] = 'pose_hold_finished'
    except (OperatorStop, KeyboardInterrupt):
        report['status'] = 'operator_finished' if report.get('holding') else 'operator_stopped'
    except BaseException as exc:
        report.update(status='stopped', error=str(exc))
        print('STOP:', exc, flush=True)
    finally:
        if bus is not None: bus.close()
        if live is not None: report['live_state_writer'] = live.close()
        report['serial_port_closed'] = True
        if folder.exists():
            (folder/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
            print('Saved:', folder, flush=True)
    return 0 if not report.get('cleanup_error') and report['status'] in ('operator_finished','pose_hold_finished','operator_stopped') else 1


if __name__ == '__main__':
    raise SystemExit(main())
