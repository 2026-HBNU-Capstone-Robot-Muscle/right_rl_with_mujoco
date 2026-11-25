"""Original 1x vertical path with bounded communication/termination repairs."""
import argparse
import contextlib
import ctypes
import datetime as dt
import hashlib
import json
from pathlib import Path
import sys
import threading
import time

import leap_measured_grip_hold_20261117 as pose
import leap_vertical_direct_grip_20261118 as original
from grip_transport_recovery_20261125 import RecoveryBus, POLICY

ROOT = Path(__file__).resolve().parent
BASE = ROOT/'right_hand_rotation_20261125/communication_repair'


def validate_repair_sources():
    manifest = json.loads((BASE/'manifest.json').read_bytes())
    for name, expected in manifest['source_hashes'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Communication repair source changed: '+name)
    return manifest


class StopRequest:
    def __init__(self):
        self.requested = threading.Event()
        self.finished = threading.Event()
        self.reason = None
        self.callback = self.kernel = None

    def check(self):
        if self.requested.is_set():
            raise pose.OperatorStop()

    def install(self):
        # Windows gives console CLOSE a limited cleanup window. This requests
        # cooperative cleanup; it cannot promise OFF after PC power loss.
        self.kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        callback_type = ctypes.WINFUNCTYPE(ctypes.c_int, ctypes.c_uint)

        def receive(event):
            if event not in (0, 1, 2, 5, 6):
                return 0
            self.reason = int(event)
            self.requested.set()
            self.finished.wait(3.)
            return 1

        self.callback = callback_type(receive)
        self.kernel.SetConsoleCtrlHandler.argtypes = [callback_type, ctypes.c_int]
        if not self.kernel.SetConsoleCtrlHandler(self.callback, True):
            raise ctypes.WinError(ctypes.get_last_error())

    def close(self):
        self.finished.set()
        if self.callback is not None:
            self.kernel.SetConsoleCtrlHandler(self.callback, False)


def terminal_for(stop):
    parent = original.Terminal

    class Terminal(parent):
        def wait(self):
            import msvcrt
            while True:
                stop.check()
                if msvcrt.kbhit():
                    key = msvcrt.getwch()
                    if key == ' ':
                        return
                    self.handle(key)
                time.sleep(.02)

        def check(self):
            stop.check()
            return super().check()

    return Terminal


def readonly_check():
    import numpy as np
    from final_grip_single_instance_20261116 import SingleInstance
    from grip_health_read_20261118 import block_once
    folder = ROOT/('comm_repair_readonly_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    report = dict(status='started', servo_writes=0, torque_changes=0, comparisons=[])
    bus = None
    try:
        with SingleInstance(ROOT/'.final_grip_pose.lock'):
            folder.mkdir()
            bus = RecoveryBus(report, lambda: None, read_only=True)
            for _ in range(3):
                for name in ('paired_sync', 'full16_unicast'):
                    began = time.perf_counter()
                    state, comm, error, details = (block_once(bus, 64, 83, pose.FIELDS)
                        if name == 'paired_sync' else bus._unicast_full_health())
                    elapsed = time.perf_counter()-began
                    report['comparisons'].append(dict(method=name, elapsed_ms=elapsed*1000,
                        communication=int(comm), device_error=int(error), details=details))
                    if comm or error or state is None:
                        raise RuntimeError('Read-only comparison failed: '+name)
                    if np.any(state['torque']) or np.any(state['error']):
                        raise RuntimeError('Read-only comparison requires healthy torque OFF')
                    time.sleep(max(0., .075-elapsed))
            report['status'] = 'readonly_comparison_complete'
    except (Exception, KeyboardInterrupt) as exc:
        report.update(status='stopped', error=str(exc))
    finally:
        if bus is not None:
            bus.close()
        report['serial_port_closed'] = True
        if folder.exists():
            (folder/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('last_outgoing_packets',)}, ensure_ascii=False))
    return 0 if report['status'] == 'readonly_comparison_complete' else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--read-only-check', action='store_true')
    args = parser.parse_args(argv)
    if args.execute and args.read_only_check:
        parser.error('Choose either operator execution or READ-only comparison')
    try:
        validate_repair_sources()
        pose.validate_sources()
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        print('NOT READY:', exc, 'No devices opened.')
        return 2
    if args.read_only_check:
        return readonly_check()
    stop = StopRequest()
    old_bus, old_terminal, old_argv = pose.PoseBus, original.Terminal, sys.argv
    try:
        pose.PoseBus = RecoveryBus
        original.Terminal = terminal_for(stop)
        sys.argv = [str(__file__)]+(['--execute'] if args.execute else [])
        if args.execute:
            stop.install()
            print('1배속 통신 복구 수정본: 기존 회전/G 경로·75ms·최대2틱·450mA/PID 유지.', flush=True)
            print('읽기 실패 시 전체16개 개별 READ로 복구 확인. 두 정상 상태/200ms 기준 유지.', flush=True)
            print('종료 확인은 운동 재시도 한도와 분리. 토크 OFF가 확인되지 않으면 그대로 실패로 기록합니다.', flush=True)
            print('창 닫기에도 정상 종료를 요청하지만 PC 전원 차단 시 OFF를 보장하지는 못합니다.', flush=True)
        return original.main()
    finally:
        stop.close()
        pose.PoseBus, original.Terminal, sys.argv = old_bus, old_terminal, old_argv


if __name__ == '__main__':
    raise SystemExit(main())
