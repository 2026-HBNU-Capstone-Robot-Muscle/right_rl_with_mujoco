"""Source-bound empty root lift; NOT a SAC path or a thumb-range calibration.

Only raw IDs 8/12/16 advance. All other goals retain the fresh OFF encoder
values, including ID3's unwrapped mode-5 reading. No homing or mode writes.
Default is offline check. --execute requires the operator's SPACE.
"""
import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import hand_motor_mapping as mapping
import sequence_flip_checked as checked

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / 'right_hand_start_20261019_1811/source_preflight.json'
SOURCE_SHA = '4fbb1da231f0e1361654c2f4cf83b3ceb2c25cd451ef8756699699dd9aadce70'
ADDITIONAL_REVIEWED_SOURCES = (
    ('right_hand_start_20261019/source_readonly.json',
     'faa0c6227b8f14dc68a3cd8d928837f6f72709c7bb303b8a34e67b53917b74d8'),
    ('right_hand_start_20261019_thumb_shift/source_preflight.json',
     'a0274a7b108f9a866c7ae680cfe321b41b3f65f5b1f0cd3f9d03303cf2531350'),
)
ROOT_INDICES = np.array([7, 11, 15])
FROZEN = np.array([i for i in range(16) if i not in ROOT_INDICES])
ROOT_TARGET = 2472
# Kinematic sensitivity review: right_hand_root_drift_20261020/screen.json.
# This is a START neighborhood only; OFF/ON and live motion guards are separate.
SOURCE_TOLERANCE = 32
STEPS, DELAY, HOLD_READS = 120, .05, 20


def source_position(path=None, digest=None):
    path = SOURCE if path is None else Path(path)
    digest = SOURCE_SHA if digest is None else digest
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != digest:
        raise ValueError('Reviewed source changed; no motion')
    record = json.loads(data)
    if (record['joint_mapping']['motor_ids_in_sim_order'] != list(mapping.SIM_TO_MOTOR_IDS)
            or list(mapping.SIM_TO_MOTOR_IDS) != [8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1]):
        raise ValueError('Mapping differs from reviewed root lift')
    if 'preflight_state' in record:
        state = record['preflight_state']
        if (record.get('write_events') != [] or not record.get('all_torque_off_verified')
                or any(any(state[k]) for k in ('torque','error','watchdog'))):
            raise ValueError('Reviewed preflight source must be read-only, torque OFF and healthy')
        return np.array(state['position'], dtype=np.int64)
    return np.array([m['position_ticks_signed'] for m in record['motors']], dtype=np.int64)


def reviewed_starts():
    """Union of individually screened neighborhoods, never per-joint mixing."""
    references = [(SOURCE, SOURCE_SHA)] + [(ROOT/p,sha) for p,sha in ADDITIONAL_REVIEWED_SOURCES]
    unique = {}
    for path,digest in references:
        unique[digest] = dict(path=str(path),sha256=digest,position=source_position(path,digest))
    return list(unique.values())


def select_start(raw):
    a = np.asarray(raw)
    if a.shape != (16,) or not np.isfinite(a).all() or np.any(a != np.rint(a)):
        raise ValueError('Invalid encoder array')
    candidates = reviewed_starts()
    selected = min(candidates,key=lambda x:np.max(np.abs(a-x['position'])))
    reference = selected['position']
    bad = np.flatnonzero(np.abs(a-reference) > SOURCE_TOLERANCE)
    if len(bad):
        details = '; '.join(f'ID{i+1} {int(reference[i])}->{int(a[i])} '
                            f'({int(a[i]-reference[i]):+d} ticks)' for i in bad)
        raise ValueError(f'Pose differs from all {len(candidates)} reviewed starts (limit {SOURCE_TOLERANCE} ticks): '
                         + details + '. No writes; current readings saved in this result for review.')
    return selected


def validate_start(raw):
    select_start(raw)
    a = np.asarray(raw)
    return a.astype(np.int64)


def commands(start):
    start = validate_start(start)
    goal = start.copy()
    goal[ROOT_INDICES] = ROOT_TARGET
    if np.any(goal[ROOT_INDICES] <= start[ROOT_INDICES]):
        raise ValueError('Root motion must be upward from reviewed start')
    out = []
    for i in range(1, STEPS+1):
        row = start.copy()
        row[ROOT_INDICES] = np.rint(start[ROOT_INDICES] +
            (goal[ROOT_INDICES]-start[ROOT_INDICES])*i/STEPS).astype(int)
        out.append(row)
    return out


class Bus(checked.Bus):
    # This narrow OFF-only validator replaces neither the shared single-turn
    # validator nor any blocked SAC loader.
    planner = staticmethod(validate_start)

    def bind(self, original):
        self.root_start = validate_start(original)
        self.last_root_command = self.root_start.copy()
        self.enable_hold_adopted = False

    def adopt_enable_hold(self, accepted):
        """Once, before root motion; bounded readback adoption, never turn wrap."""
        accepted = validate_start(accepted)
        if self.enable_hold_adopted or np.any(np.abs(accepted-self.root_start)>4):
            raise ValueError('Enable hold changed beyond four ticks or was already adopted')
        self.root_start = accepted.copy()
        self.last_root_command = accepted.copy()
        self.enable_hold_adopted = True

    def write(self, addr, size, values, verify=True):
        if addr == 116:
            v = np.asarray(values)
            if not hasattr(self, 'root_start'):
                raise ValueError('No reviewed root-start binding')
            if (v.shape != (16,) or not np.isfinite(v).all() or np.any(v != np.rint(v))
                    or not np.array_equal(v[FROZEN], self.root_start[FROZEN])
                    or np.any(v[ROOT_INDICES] < self.last_root_command[ROOT_INDICES])
                    or np.any(v[ROOT_INDICES] > ROOT_TARGET)
                    or np.any(v[ROOT_INDICES]-self.last_root_command[ROOT_INDICES] > 5)):
                raise ValueError('Goal violates root-only frozen-joint/step envelope')
            self.last_root_command = v.copy()
        return super().write(addr, size, values, verify=verify)


def check_off(state, original, *, configured=False):
    if (np.any(state['torque']) or np.any(state['error']) or np.any(state['watchdog'])
            or np.any(state['voltage'] < 38) or np.any(state['voltage'] > 65)
            or np.any(state['temperature'] > 45)
            or np.any(np.abs(state['position']-original) > 4)):
        raise RuntimeError('OFF state/drift/voltage check failed; no enable')
    if configured:
        differences=[]
        for name,delta in (('goal-original',state['goal']-original),
                           ('goal-position',state['goal']-state['position'])):
            for i in np.flatnonzero(np.abs(delta)>4):
                differences.append(f'{name} ID{i+1} {int(delta[i]):+d} ticks')
        expected=dict(current_cap=checked.CAP,acceleration=np.zeros(16),velocity=np.zeros(16),**checked.GAINS)
        for name,values in expected.items():
            for i in np.flatnonzero(state[name]!=values):
                differences.append(f'{name} ID{i+1}: expected {int(values[i])}, read {int(state[name][i])}')
        if differences:
            raise RuntimeError('OFF preload/profile mismatch: '+'; '.join(differences)+'; no enable')


def run(bus, initial, report, *, cancel=checked.check_cancel,
        sleep=time.sleep, now=time.monotonic):
    original = validate_start(initial['position'])
    selected = select_start(original)
    path = commands(original)
    check_off(initial, original)
    bus.bind(original)
    report.update(initial={k:v.tolist() for k,v in initial.items()},
                  source_sha256=selected['sha256'], selected_start_source=selected['path'],
                  start_difference_ticks=(original-selected['position']).tolist(),
                  root_goal_ticks=ROOT_TARGET, moving_motor_ids=[8,12,16],
                  frozen_motor_ids=(FROZEN+1).tolist(), id3_held_raw=int(original[2]),
                  settings_restored=False, all_torque_off_verified=False)
    # Fresh state before the first configuration write, not just during preflight.
    cancel()
    check_off(bus.snapshot(), original)
    touched = False
    try:
        touched = True
        report['phase'] = 'configure_off'
        for addr,key in ((80,'d_gain'), (82,'i_gain'), (84,'p_gain')):
            bus.write(addr, 2, checked.GAINS[key])
        bus.write(102, 2, checked.CAP)
        bus.write(108, 4, np.zeros(16, dtype=int))
        bus.write(112, 4, np.zeros(16, dtype=int))
        # OFF position/goal may differ by a tick between transactions. Do not
        # retry the write. Verify fresh full snapshots with a bounded envelope.
        bus.write(116, 4, original, verify=False)
        previous = None
        for _ in range(2):
            cancel()
            state = bus.snapshot()
            report.setdefault('off_preload_snapshots', []).append({k:v.tolist() for k,v in state.items()})
            check_off(state, original, configured=True)
            if previous is not None and any(np.any(np.abs(state[k]-previous[k])>1) for k in ('goal','position')):
                raise RuntimeError('OFF preload not stable within one tick across fresh reads; no enable')
            previous = state
            sleep(.02)
        bus.write(98, 1, np.full(16,25))
        cancel()
        bus.write(64, 1, np.ones(16, dtype=int))
        report['enabled_at'] = now()
        report['phase'] = 'enable_hold'
        previous = None
        for _ in range(2):
            cancel()
            state = bus.snapshot()
            report.setdefault('enable_snapshots', []).append({k:v.tolist() for k,v in state.items()})
            checked.health(state, state['goal'])
            if (np.any(np.abs(state['position']-original)>4)
                    or np.any(np.abs(state['goal']-original)>4)
                    or np.any(np.abs(state['goal']-state['position'])>4)):
                raise RuntimeError('Position changed on enable, including raw turn count; OFF')
            if previous is not None and any(np.any(np.abs(state[k]-previous[k])>1) for k in ('goal','position')):
                raise RuntimeError('Enable hold not stable within one tick; OFF')
            previous = state
            sleep(.02)
        command = state['goal'].copy()
        bus.adopt_enable_hold(command)
        path = commands(command)
        frozen_reference = command.copy()
        report.update(accepted_enable_goal=command.tolist(),
                      enable_goal_shift_ticks=(command-original).tolist(),
                      id3_held_raw=int(command[2]))
        # A single deliberate ON hold command establishes exact subsequent
        # goal checks. This is not a failed-packet retry or a motion target.
        bus.write(116,4,command)

        def observe(phase):
            cancel()
            if now()-report['enabled_at'] > 20:
                raise RuntimeError('Root check exceeded 20 seconds; OFF')
            state, errors = checked.sample(bus, command, report, phase)
            if np.any(np.abs(state['position'][FROZEN]-frozen_reference[FROZEN]) > 20):
                raise RuntimeError('A held joint moved over 20 ticks; no root advance')
            return state, errors

        # Read before EVERY next target; no stale packet can advance the path.
        for i,target in enumerate(path, 1):
            report['phase'] = 'roots_only'
            state, errors = observe('roots_only')
            bus.write(116, 4, target, verify=False)
            command = target.copy()
            if i % 20 == 0:
                print(f'roots_only: {i}/{STEPS}; previous-goal tracking {max(errors):.3f}rad', flush=True)
            sleep(DELAY)
        report['phase'] = 'fixed_hold'
        for _ in range(HOLD_READS):
            state, errors = observe('fixed_hold')
            sleep(DELAY)
        report.update(final_on_state={k:v.tolist() for k,v in state.items()},
                      status='empty_root_lift_complete')
    finally:
        if touched:
            bus.off()
            report['all_torque_off_verified'] = True
            bus.write(98, 1, np.zeros(16,dtype=int))
            for addr,size,key in ((102,2,'current_cap'),(108,4,'acceleration'),(112,4,'velocity'),
                                  (98,1,'watchdog'),(80,2,'d_gain'),(82,2,'i_gain'),(84,2,'p_gain')):
                bus.write(addr, size, initial[key])
            report['settings_restored'] = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    references = reviewed_starts()
    print('EMPTY ROOT CHECK v5: reviewed start drift <=32 ticks; roots ID8/12/16 ->2472, then OFF.', flush=True)
    print('Preload/enable shift <=4 ticks; two stable fresh reads; no goal write retry.', flush=True)
    print(f'{len(references)} reviewed starts available; fresh readings choose the matching start.',flush=True)
    print('All other goals hold fresh OFF readings; ID3 is NOT wrapped or moved. No cube/SAC.', flush=True)
    if not args.execute:
        print('OFFLINE ONLY: no port.')
        for entry in references:
            source = entry['position'];path = commands(source)
            print('Reviewed:',Path(entry['path']).parent.name,'start:',source.tolist(),'final:',path[-1].tolist())
        return 0
    report = dict(timestamp=dt.datetime.now().astimezone().isoformat(), status='not_started',
                  reviewed_source_sha256=[x['sha256'] for x in references], joint_mapping=mapping.metadata(),
                  start_tolerance_ticks=SOURCE_TOLERANCE,
                  start_sensitivity_review='right_hand_root_drift_20261020/screen.json',
                  policy_used=False, physical_flip_verified=False,
                  all_torque_off_verified=False, hardware_authorized=False)
    bus = None
    opened = False
    try:
        print('EMPTY hand; leave fingers as they are. Secure base and clear fingers/cables.', flush=True)
        print('450mA, original sequence PID. SPACE here starts checks/countdown; Q/Esc/Ctrl+C stops.', flush=True)
        import msvcrt
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getwch()
                if key in ('q', 'Q', '\x1b', '\x03'):
                    raise KeyboardInterrupt('User stop')
                if key == ' ':
                    break
            time.sleep(.03)
        report['hardware_authorized'] = True
        import dynamixel_sdk as sdk
        bus = Bus(sdk, report)
        if not bus.port.openPort():
            raise RuntimeError('Cannot open COM7')
        opened = True
        if not bus.port.setBaudRate(3000000):
            raise RuntimeError('Cannot set baud')
        initial = bus.preflight()
        check_off(initial, initial['position'])
        selected=select_start(initial['position'])
        report['selected_start_source']=selected['path']
        report['source_sha256']=selected['sha256']
        print('Matched reviewed start:',Path(selected['path']).parent.name,flush=True)
        print('Root travel (degrees):',np.round((ROOT_TARGET-initial['position'][ROOT_INDICES])*360/4096,2).tolist(),flush=True)
        for n in range(3,0,-1):
            print(f'Starting in {n}s; EMPTY hand.', flush=True)
            for _ in range(20):
                checked.check_cancel()
                time.sleep(.05)
        fresh = bus.preflight()
        if np.any(np.abs(fresh['position']-initial['position']) > 4):
            raise RuntimeError('Pose changed during countdown; no writes')
        run(bus, fresh, report)
        print('Root lift complete. Torque OFF verified; settings restored.', flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        report.update(status='stopped', error=str(exc) or 'Interrupted', failed_phase=report.get('phase'))
        print('STOP:', report['error'], flush=True)
    finally:
        if bus is not None and opened:
            try:
                torque = bus.block(64,1,[('torque',64,1)])['torque']
                report['all_torque_off_verified'] = bool(np.all(torque == 0))
            except Exception as exc:
                report.update(all_torque_off_verified=False, final_check_error=str(exc))
            if not report['all_torque_off_verified']:
                print('CUT MOTOR POWER NOW: torque OFF unverified.', flush=True)
            bus.port.closePort()
        stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        for name in (f'leap_roots_empty_{stamp}.json', 'leap_roots_empty_result.json'):
            (ROOT/name).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print('Saved:', ROOT/'leap_roots_empty_result.json', flush=True)
    return 0 if report['status'] == 'empty_root_lift_complete' and report['all_torque_off_verified'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
