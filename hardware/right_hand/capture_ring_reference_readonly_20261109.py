"""Save the operator's current ring pose: Protocol 2 PING/READ only.

Torque is observed without requiring or changing its state. No motor program
is imported and every outgoing servo instruction is restricted to PING/READ.
"""
from pathlib import Path
import datetime as dt
import json
import math
import time

ROOT = Path(__file__).resolve().parent
RING_ORDER = (13, 15, 14, 16)
ORDER = RING_ORDER + tuple(i for i in range(1, 17) if i not in RING_ORDER)


def main():
    import dynamixel_sdk as sdk
    from serial.tools.list_ports import comports

    now = dt.datetime.now(dt.timezone.utc).astimezone(dt.timezone(dt.timedelta(hours=9)))
    folder = ROOT / ('ring_reference_readonly_' + now.strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir()
    report = {'timestamp_kst': now.isoformat(), 'status': 'started', 'baud': 1000000,
              'servo_writes': 0, 'torque_changes': 0, 'instruction_counts': {},
              'inventory': [], 'snapshots': [], 'errors': [],
              'ring_tip_to_palm_ids': list(RING_ORDER),
              'reference_only_not_a_motion_goal': True}
    port = None

    def signed(value, width):
        return value - (1 << width) if value & (1 << (width - 1)) else value

    try:
        devices = [p for p in comports() if p.vid == 1027 and p.pid == 24596
                   and p.serial_number == 'FTBENPSAA']
        if len(devices) != 1:
            raise RuntimeError('Known robot USB adapter missing or ambiguous')
        report['port'] = devices[0].device
        port = sdk.PortHandler(devices[0].device)
        packet = sdk.PacketHandler(2.)
        if not port.openPort() or not port.setBaudRate(1000000):
            raise RuntimeError('Cannot open the known adapter at 1 Mbps')
        original = port.writePort

        def readonly(data):
            if len(data) < 8 or data[7] not in (1, 2):
                raise RuntimeError('Non-read servo instruction blocked')
            if data[7] == 2 and list(data[8:12]) != [64, 0, 83, 0]:
                raise RuntimeError('Read outside the documented health/position block blocked')
            key = str(data[7])
            report['instruction_counts'][key] = report['instruction_counts'].get(key, 0) + 1
            return original(data)

        port.writePort = readonly
        supported = []
        for ident in ORDER:
            model, comm, error = packet.ping(port, ident)
            report['inventory'].append({'id': ident, 'model': int(model), 'communication': int(comm), 'device_error': int(error)})
            if comm or error or model != 1240:
                report['errors'].append(f'ID{ident}: PING/model check failed ({model}, {comm}, {error})')
            else:
                supported.append(ident)
        for sample in range(2):
            rows = []
            for ident in supported:
                data, comm, error = packet.readTxRx(port, ident, 64, 83)
                if comm or error or len(data) != 83:
                    report['errors'].append(f'ID{ident}: READ failed ({comm}, {error}, length={len(data)})')
                    continue

                def value(address, size):
                    return int.from_bytes(bytes(data[address - 64:address - 64 + size]), 'little')

                ticks = signed(value(132, 4), 32)
                rows.append({'id': ident, 'torque_enabled': value(64, 1), 'hardware_error': value(70, 1),
                             'watchdog': value(98, 1), 'position_ticks': ticks,
                             'qsim_rad': (ticks - 2048) * 2 * math.pi / 4096,
                             'qsim_deg': (ticks - 2048) * 360 / 4096,
                             'goal_ticks_readonly': signed(value(116, 4), 32),
                             'present_current_ma': signed(value(126, 2), 16),
                             'voltage_v': value(144, 2) / 10, 'temperature_c': value(146, 1)})
            report['snapshots'].append({'sample': sample, 'monotonic_s': time.perf_counter(), 'motors': rows})
        by_sample = [{row['id']: row for row in snap['motors']} for snap in report['snapshots']]
        complete = all(all(ident in snap for ident in RING_ORDER) for snap in by_sample)
        if complete:
            report['ring_reference'] = []
            for ident in RING_ORDER:
                row = dict(by_sample[-1][ident])
                row['drift_ticks'] = row['position_ticks'] - by_sample[0][ident]['position_ticks']
                report['ring_reference'].append(row)
            report['ring_maximum_drift_ticks'] = max(abs(row['drift_ticks']) for row in report['ring_reference'])
            report['ring_stationary_within_3_ticks'] = report['ring_maximum_drift_ticks'] <= 3
            report['status'] = 'ring_reference_saved'
        else:
            report['status'] = 'ring_reference_incomplete'
        report['all_16_saved_twice'] = all(len(snap) == 16 for snap in by_sample)
        report['all_sampled_torque_off'] = all(row['torque_enabled'] == 0 for snap in by_sample for row in snap.values()) if supported else None
    except (Exception, KeyboardInterrupt) as exc:
        report['status'] = 'stopped'
        report['errors'].append(str(exc))
    finally:
        if port is not None and getattr(port, 'ser', None) is not None:
            port.closePort()
        report['serial_port_closed'] = True
        (folder / 'result.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps({'folder': folder.name, 'status': report['status'],
                      'ring': report.get('ring_reference'), 'errors': report['errors'],
                      'all_16_saved_twice': report.get('all_16_saved_twice'),
                      'all_sampled_torque_off': report.get('all_sampled_torque_off'),
                      'instructions': report['instruction_counts'], 'servo_writes': 0,
                      'torque_changes': 0, 'serial_port_closed': True}, ensure_ascii=False))


if __name__ == '__main__':
    main()
