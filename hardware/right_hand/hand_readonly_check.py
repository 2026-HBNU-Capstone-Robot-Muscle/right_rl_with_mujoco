"""One-shot motor inspection: Protocol 2 PING/READ only, no WRITE or torque changes.

Control table references:
https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/
https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/
Unsupported models are reported without interpreting their control tables.
"""
import argparse
import datetime
import json
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work/Leap_Test/deploy'))
from hand_motor_mapping import hw_to_sim, SIM2ARR, SIM_JOINT_NAMES, metadata as mapping_metadata
import limits

MODELS = {1220: 'XC330-T288', 1240: 'XC330-M288'}


def signed32(value):
    return value - (1 << 32) if value & (1 << 31) else value


DRIVE_REGISTERS = (
    ('pwm_limit_raw',36,2),('current_limit_ma',38,2),
    ('max_position_limit',48,4),('min_position_limit',52,4),
    ('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),
    ('goal_pwm_raw',100,2),('goal_current_raw',102,2),
    ('profile_acceleration',108,4),('profile_velocity',112,4))

def read_drive_settings(read,motor):
    values={}
    for name,address,size in DRIVE_REGISTERS:
        first=read(motor,address,size);second=read(motor,address,size)
        if first!=second:raise RuntimeError(f'ID {motor} unstable {name}: {first}, {second}; read-only inspection stopped')
        values[name]=first
    values['pwm_limit_percent_approx']=values['pwm_limit_raw']*.113
    return values

def require_serial_port(device, report, enumerate_ports=None):
    """Inventory only: never opens a port or selects a replacement automatically."""
    if enumerate_ports is None:
        from serial.tools import list_ports
        enumerate_ports = list_ports.comports
    ports = list(enumerate_ports())
    report['available_ports'] = [dict(device=p.device, description=p.description,
                                     vid=p.vid, pid=p.pid) for p in ports]
    if device.upper() not in {p.device.upper() for p in ports}:
        available = ', '.join(p.device for p in ports) or '(none)'
        raise RuntimeError(f'{device} not present. Available: {available}. '
                           'Reconnect the motor USB adapter; motor power alone does not create a COM port. '
                           'No replacement port selected.')


def inspect_bus(sdk, device, report, *, drive_settings=False):
    port = sdk.PortHandler(device)
    packet = sdk.PacketHandler(2.0)
    opened = False

    def read(motor, addr, size):
        method = {1: packet.read1ByteTxRx, 2: packet.read2ByteTxRx,
                  4: packet.read4ByteTxRx}[size]
        value, comm, error = method(port, motor, addr)
        if comm != sdk.COMM_SUCCESS or error:
            raise RuntimeError(f'ID {motor} read {addr}: communication={comm}, device_error={error}')
        return int(value)

    try:
        if not port.openPort():
            raise RuntimeError(f'Cannot open {device}; close other motor programs')
        opened = True
        report['port_opened'] = True
        if not port.setBaudRate(3000000):
            raise RuntimeError('Cannot set host serial baud rate to 3 Mbps')
        for motor in range(1, 17):
            row = {'id': motor}
            report['motors'].append(row)
            model, comm, error = packet.ping(port, motor)
            row.update(model_number=int(model), ping_comm=int(comm), ping_error=int(error))
            if comm != sdk.COMM_SUCCESS or error:
                raise RuntimeError(f'ID {motor} ping failed: communication={comm}, device_error={error}')
            if model not in MODELS:
                raise RuntimeError(f'ID {motor}: unverified model {model}; stopped before control-table reads')
            row['model'] = MODELS[model]
            row['torque_enabled'] = read(motor, 64, 1)
            if row['torque_enabled'] != 0:
                raise RuntimeError(f'ID {motor} TORQUE IS ON. Stop and disconnect motor power. This program will NOT change torque.')
            row['hardware_error'] = read(motor, 70, 1)
            if row['hardware_error']:
                raise RuntimeError(f'ID {motor}: hardware error {row["hardware_error"]}; disconnect motor power')
            row['bus_watchdog_raw'] = read(motor, 98, 1)
            for name, addr, size in [('firmware', 6, 1), ('drive_mode', 10, 1),
                                     ('operating_mode', 11, 1), ('homing_offset_raw', 20, 4),
                                     ('position_raw', 132, 4), ('voltage_raw', 144, 2),
                                     ('temperature_c', 146, 1)]:
                row[name] = read(motor, addr, size)
            row['position_ticks_signed'] = signed32(row['position_raw'])
            row['hw_angle_rad'] = row['position_ticks_signed'] * (2*np.pi/4096)
            row['voltage_v'] = row['voltage_raw'] / 10
            if drive_settings:row['drive_settings']=read_drive_settings(read,motor)
            print(f'ID {motor:2d} {row["model"]}: torque=OFF, watchdog={row["bus_watchdog_raw"]}, '
                  f'angle={row["hw_angle_rad"]:.3f} rad, '
                  f'{row["voltage_v"]:.1f} V, {row["temperature_c"]} C', flush=True)
            if drive_settings:
                s=row['drive_settings']
                print(f'  Mode={row["operating_mode"]}; PWM limit={s["pwm_limit_raw"]} (~{s["pwm_limit_percent_approx"]:.1f}%); '
                      f'Goal PWM={s["goal_pwm_raw"]}; current limit={s["current_limit_ma"]}mA; '
                      f'PID={s["p_gain"]}/{s["i_gain"]}/{s["d_gain"]}',flush=True)
        # A second torque check detects changes during the sequential snapshot.
        for row in report['motors']:
            row['torque_enabled_end'] = read(row['id'], 64, 1)
            if row['torque_enabled_end'] != 0:
                raise RuntimeError('Torque changed during inspection; disconnect motor power')
        hw = np.array([m['hw_angle_rad'] for m in report['motors']])
        sim = hw_to_sim(hw)
        report['sim_angles_rad'] = sim.tolist()
        report['joint_mapping'] = mapping_metadata()
        report['joint_map'] = [{'motor_id': int(SIM2ARR[i])+1, 'joint': name,
                                'angle_rad': float(sim[i]),
                                'outside_configured_range': bool(sim[i] < limits.SAFE_LOWERS[i] or sim[i] > limits.SAFE_UPPERS[i])}
                               for i, name in enumerate(SIM_JOINT_NAMES)]
        report['status'] = 'read_complete_not_motion_approval'
    finally:
        # SDK closePort dereferences ser even when opening failed and ser is None.
        # Also close a partially opened handle, without masking the original error.
        original_error = sys.exc_info()[0] is not None
        if opened or getattr(port, 'ser', None) is not None:
            try:
                port.closePort()  # Host serial close only; no motor writes.
            except Exception as exc:
                report['port_close_error'] = str(exc)
                if not original_error:
                    raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM7')
    parser.add_argument('--drive-settings',action='store_true',help='Also read output limits/PID twice without changing them')
    args = parser.parse_args()
    report = {'status': 'not_started', 'timestamp': datetime.datetime.now().astimezone().isoformat(),
              'port': args.port, 'port_opened': False, 'hardware_authorized': False,
              'servo_write_commands_sent': 0, 'torque_changes_requested': 0, 'motors': []}
    code = 1
    try:
        import dynamixel_sdk as sdk
        print('READ ONLY: PING/READ only. No target, torque, gain, mode, or EEPROM writes.', flush=True)
        print('Other motor programs must be closed. Cube removed; keep fingers clear.', flush=True)
        require_serial_port(args.port, report)
        inspect_bus(sdk, args.port, report,drive_settings=args.drive_settings)
        print('16 motors read. This is NOT approval to run the policy.', flush=True)
        for row in report['joint_map']:
            print(f'ID {row["motor_id"]:2d} {row["joint"]:8s}: {row["angle_rad"]:+.3f} rad'
                  + (' [outside configured range; no correction sent]' if row['outside_configured_range'] else ''))
        code = 0
    except (Exception, KeyboardInterrupt) as exc:
        report.update(status='stopped', error=str(exc) or 'Interrupted')
        print(f'STOP: {report["error"]}', flush=True)
    finally:
        path = ROOT / ('leap_drive_readonly_result.json' if args.drive_settings else 'hand_readonly_check_result.json')
        path.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(f'Saved: {path}', flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
