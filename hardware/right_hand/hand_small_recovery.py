"""Explicitly armed, empty-hand, one-joint-at-a-time preparation. NOT policy control.

RAM only; no mode/zero/EEPROM/gain edits. XC330-M288 control table:
https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/
"""
import datetime
import json
import sys
import time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work/Leap_Test/deploy'))
from leap_hw import OFFSET, SIM2ARR, hw_to_sim
import limits
from hand_readonly_check import signed32

TICK = 2*np.pi/4096
ALLOWED_IDS = {7, 8, 11, 12, 15}
MAX_MOVE = .15
TARGET = -.18
CURRENT_MA = 60


def make_plan(raw):
    if len(raw) != 16 or not all(0 <= x <= 4095 for x in raw):
        raise ValueError('Expected 16 single-turn positions; no wrapping or zero correction permitted')
    q = hw_to_sim(np.asarray(raw)*TICK)
    plan = []
    for i, angle in enumerate(q):
        motor = int(SIM2ARR[i])+1
        if angle > limits.SAFE_UPPERS[i]:
            raise ValueError(f'ID {motor}: above range; outside this test')
        if angle >= limits.SAFE_LOWERS[i]:
            continue
        if motor not in ALLOWED_IDS or not np.isclose(limits.SAFE_LOWERS[i], -.2):
            raise ValueError(f'ID {motor}: unexpected out-of-range joint')
        target = int(np.ceil((TARGET + OFFSET)/TICK))
        delta = (target - raw[motor-1])*TICK
        if not 0 < delta <= MAX_MOVE:
            raise ValueError(f'ID {motor}: needs {delta:.3f} rad; exceeds small-test allowance')
        plan.append({'id': motor, 'start_raw': int(raw[motor-1]), 'target_raw': target,
                     'start_sim_rad': float(angle), 'delta_rad': float(delta)})
    return plan


class Bus:
    def __init__(self, sdk):
        self.sdk = sdk
        self.port = sdk.PortHandler('COM7')
        self.packet = sdk.PacketHandler(2.0)
        self.write_events = []

    def read(self, motor, addr, size=1):
        fn = {1: self.packet.read1ByteTxRx, 2: self.packet.read2ByteTxRx,
              4: self.packet.read4ByteTxRx}[size]
        v, comm, error = fn(self.port, motor, addr)
        if comm != self.sdk.COMM_SUCCESS or error:
            raise RuntimeError(f'ID {motor} READ {addr}: comm={comm}, error={error}')
        return int(v)

    def write(self, motor, addr, value, size):
        if motor not in ALLOWED_IDS or (addr, size) not in {(64,1),(98,1),(102,2),(108,4),(112,4),(116,4)}:
            raise ValueError('Write outside approved small-test register/ID scope')
        fn = {1: self.packet.write1ByteTxRx, 2: self.packet.write2ByteTxRx,
              4: self.packet.write4ByteTxRx}[size]
        event = {'motor_id': motor, 'address': addr, 'size': size,
                 'requested': int(value), 'monotonic_s': time.monotonic()}
        self.write_events.append(event)
        comm, error = fn(self.port, motor, addr, int(value))
        event.update(comm=int(comm), device_error=int(error))
        if comm != self.sdk.COMM_SUCCESS or error:
            raise RuntimeError(f'ID {motor} WRITE {addr}: comm={comm}, error={error}')
        observed = self.read(motor, addr, size)
        event['readback'] = observed
        if observed != value:
            raise RuntimeError(f'ID {motor} register {addr} readback mismatch: wrote={value}, read={observed}')

    def preflight(self):
        raw = []
        for motor in range(1, 17):
            model, comm, error = self.packet.ping(self.port, motor)
            if comm != self.sdk.COMM_SUCCESS or error or model != 1240:
                raise RuntimeError(f'ID {motor}: model/communication changed')
            expected = {64: 0, 70: 0, 11: 5, 10: 0, 98: 0}
            for addr, value in expected.items():
                if self.read(motor, addr) != value:
                    raise RuntimeError(f'ID {motor}: unexpected state at {addr}; no automatic correction')
            if self.read(motor, 6) < 38 or self.read(motor, 20, 4) != 0:
                raise RuntimeError(f'ID {motor}: unsupported firmware/offset')
            if self.read(motor, 146) > 45 or not 35 <= self.read(motor, 144, 2) <= 65:
                raise RuntimeError(f'ID {motor}: temperature/voltage outside conservative test bounds')
            if self.read(motor, 38, 2) < CURRENT_MA:
                raise RuntimeError(f'ID {motor}: configured current limit below test setting')
            raw.append(signed32(self.read(motor, 132, 4)))
        return make_plan(raw)


def check_cancel():
    import msvcrt
    if msvcrt.kbhit():
        if msvcrt.getwch().lower() in ('q', '\x1b'):
            raise KeyboardInterrupt('User stop')


def move_one(bus, item, log, cancel=check_cancel, now=time.monotonic, sleep=time.sleep):
    motor = item['id']
    saved = {}
    touched = False
    try:
        log['phase'] = 'position_recheck_torque_off'
        if bus.read(motor, 64) != 0:
            raise RuntimeError('Torque was enabled externally; stop')
        start = signed32(bus.read(motor, 132, 4))
        target = item['target_raw']
        log.update(start_raw=start, target_raw=target)
        if abs(start-item['start_raw'])*TICK > .025 or not 0 < (target-start)*TICK <= MAX_MOVE:
            raise RuntimeError('Position changed since confirmation; restart read-only check')
        for addr, size in ((98,1),(102,2),(108,4),(112,4)):
            saved[addr] = (bus.read(motor, addr, size), size)
        log['saved_ram'] = {str(a): v[0] for a,v in saved.items()}
        touched = True
        log['phase'] = 'configure_ram_torque_off'
        # Prepare the current position BEFORE enabling this single motor.
        bus.write(motor, 102, CURRENT_MA, 2)
        bus.write(motor, 108, 5, 4)
        bus.write(motor, 112, 5, 4)
        log['phase'] = 'preload_current_position_torque_off'
        bus.write(motor, 116, start, 4)
        log['phase'] = 'watchdog_and_position_recheck_torque_off'
        bus.write(motor, 98, 25, 1)  # 500ms bus watchdog; NOT a torque-off guarantee.
        if abs(signed32(bus.read(motor, 132, 4))-start)*TICK > .015:
            raise RuntimeError('Hand moved during preparation')
        cancel()
        log['phase'] = 'enable_torque'
        bus.write(motor, 64, 1, 1)
        log['phase'] = 'bounded_ramp'
        began = now()
        command = start
        settled = None
        log['samples'] = []
        while True:
            cancel()
            if now()-began > 12:
                raise RuntimeError('Movement timeout; no automatic current increase')
            actual = signed32(bus.read(motor, 132, 4))
            if not start-15 <= actual <= target+15:
                raise RuntimeError('Unexpected position/overshoot')
            if abs(actual-command)*TICK > .075:
                raise RuntimeError('Tracking error: insufficient force or obstruction; stopping')
            if bus.read(motor, 70) != 0 or bus.read(motor, 64) != 1:
                raise RuntimeError('Motor fault/torque changed')
            # At most two encoder ticks per iteration, never catch up after lag.
            command = min(command+2, target)
            bus.write(motor, 116, command, 4)
            log['samples'].append({'t': now()-began, 'command_raw': command, 'actual_raw': actual})
            if command == target:
                settled = now() if settled is None else settled
                if now()-settled >= 1:
                    if abs(actual-target)*TICK > .02 or actual*TICK-OFFSET < -.2:
                        raise RuntimeError('Target not reached; no stronger retry')
                    log['status'] = 'target_reached_before_torque_off'
                    break
            sleep(.05)
    except BaseException as exc:
        log['failed_phase'] = log.get('phase')
        log['error'] = str(exc) or type(exc).__name__
        raise
    finally:
        if touched:
            # A failed enable can still have reached the servo: always attempt OFF.
            try:
                bus.write(motor, 64, 0, 1)
                log['torque_off_verified'] = True
            except Exception as exc:
                log['torque_off_verified'] = False
                raise RuntimeError('TORQUE OFF NOT VERIFIED: CUT MOTOR POWER NOW') from exc
            # Restore temporary RAM settings only AFTER verified torque OFF.
            bus.write(motor, 98, 0, 1)
            for addr in (102,108,112,98):
                value, size = saved[addr]
                bus.write(motor, addr, value, size)
            log['settings_restored'] = True


def main():
    report = {'timestamp': datetime.datetime.now().astimezone().isoformat(),
              'status': 'not_started', 'policy_used': False, 'port': 'COM7',
              'current_ma': CURRENT_MA, 'max_command_change_rad': MAX_MOVE, 'moves': []}
    bus = None
    opened = False
    try:
        import dynamixel_sdk as sdk
        bus = Bus(sdk)
        if not bus.port.openPort():
            raise RuntimeError('COM7 cannot open; close other motor programs')
        opened = True
        if not bus.port.setBaudRate(3000000):
            raise RuntimeError('Cannot set host baud')
        plan = bus.preflight()
        report['plan'] = plan
        print('EMPTY HAND ONLY. One motor at a time. Q/Ctrl+C stops; keep power switch accessible.', flush=True)
        for item in plan:
            print(f'ID {item["id"]}: {item["start_sim_rad"]:+.3f} -> about -0.18 rad (+{item["delta_rad"]:.3f})', flush=True)
        if not plan:
            report['status'] = 'no_movement_needed'
            return 0
        if input('Remove cube, keep fingers clear. Type MOVE then Enter to start: ').strip() != 'MOVE':
            report['status'] = 'cancelled_before_writes'
            return 0
        check_cancel()
        fresh = bus.preflight()
        if [x['id'] for x in fresh] != [x['id'] for x in plan] or any(abs(a['start_raw']-b['start_raw'])*TICK > .025 for a,b in zip(plan,fresh)):
            raise RuntimeError('Confirmed plan changed; no movement')
        print('Starting in 3 seconds. Q stops.', flush=True)
        for _ in range(60):
            check_cancel()
            time.sleep(.05)
        for item in fresh:
            print(f'Moving ID {item["id"]} only...', flush=True)
            log = {'id': item['id'], 'status': 'started'}
            report['moves'].append(log)
            move_one(bus, item, log)
            print(f'ID {item["id"]}: target checked, torque OFF.', flush=True)
        report['status'] = 'small_test_complete_not_policy_test'
    except (Exception, KeyboardInterrupt) as exc:
        report.update(status='stopped', error=str(exc) or 'Interrupted')
        print('STOP: '+report['error'], flush=True)
    finally:
        if bus is not None and opened:
            report['write_events'] = bus.write_events
            try:
                states = {str(i): bus.read(i,64) for i in range(1,17)}
                report['final_torque_states'] = states
                report['all_torque_off_verified'] = all(v == 0 for v in states.values())
                if not report['all_torque_off_verified']:
                    print('CUT MOTOR POWER: some torque remains ON.', flush=True)
            except Exception as exc:
                report['all_torque_off_verified'] = False
                report['final_check_error'] = str(exc)
                print('CUT MOTOR POWER: final torque state cannot be verified.', flush=True)
            bus.port.closePort()
        path = ROOT / 'hand_small_recovery_result.json'
        path.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(f'Saved: {path}', flush=True)
        print('Torque release may let fingers relax again. Do not repeat automatically.', flush=True)
    return 0 if report['status'] == 'small_test_complete_not_policy_test' and report.get('all_torque_off_verified') else 1


if __name__ == '__main__':
    raise SystemExit(main())
