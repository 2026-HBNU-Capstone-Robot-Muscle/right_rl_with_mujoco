"""Observe ID12 without writes; cannot by itself prove write persistence.

No motion driver imported. Protocol PING/READ only, including during cleanup.
"""
import datetime
import json
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def collect(sdk, report, sleep=time.sleep):
    port, packet = sdk.PortHandler('COM7'), sdk.PacketHandler(2.0)
    report['samples'] = []
    report['read_events'] = []

    def read(motor, address, size=1):
        fn = {1:packet.read1ByteTxRx, 2:packet.read2ByteTxRx, 4:packet.read4ByteTxRx}[size]
        value, comm, error = fn(port, motor, address)
        report['read_events'].append({'id':motor, 'address':address, 'size':size,
                                     'value':int(value), 'comm':int(comm), 'device_error':int(error)})
        if comm != sdk.COMM_SUCCESS or error:
            raise RuntimeError(f'ID {motor} READ {address}: comm={comm}, error={error}')
        return int(value)

    def require_off(motor):
        state = read(motor,64)
        if state != 0:
            raise RuntimeError(f'ID {motor}: torque ON. Cut motor power; no automatic torque change.')
        return state

    try:
        if not port.openPort() or not port.setBaudRate(3000000):
            raise RuntimeError('Cannot open COM7 at 3 Mbps')
        for motor in range(1,17):
            model, comm, error = packet.ping(port,motor)
            if comm != sdk.COMM_SUCCESS or error or model != 1240:
                raise RuntimeError(f'ID {motor}: model={model}, comm={comm}, error={error}')
            require_off(motor)
        report['id12_configuration'] = {
            name:read(12,addr,size) for name,addr,size in (
                ('firmware',6,1), ('drive_mode',10,1), ('operating_mode',11,1),
                ('homing_offset_raw',20,4), ('startup_configuration',60,1),
                ('hardware_error',70,1), ('watchdog',98,1), ('goal_current_raw',102,2),
                ('voltage_raw',144,2), ('temperature_c',146,1))}
        if report['id12_configuration']['hardware_error'] != 0:
            raise RuntimeError('ID12 hardware error; stop and cut motor power')
        began = time.monotonic()
        for k in range(20):
            require_off(12)
            a = read(12,116,4)
            p = read(12,132,4)
            b = read(12,116,4)
            row = {'sample':k, 'time_s':time.monotonic()-began,
                   'goal_before_raw':a, 'present_raw':p, 'goal_after_raw':b}
            report['samples'].append(row)
            print(f'{k+1:02d}/20: goal {a} -> {b}; present {p}; torque OFF', flush=True)
            sleep(.1)
        report['final_torque_states'] = {str(i):require_off(i) for i in range(1,17)}
        report['all_torque_off_verified'] = True
        report['status'] = 'read_complete_not_write_validation'
    finally:
        port.closePort()


def main():
    report = {'timestamp':datetime.datetime.now().astimezone().isoformat(), 'port':'COM7',
              'status':'not_started', 'servo_write_commands_sent':0,
              'all_torque_off_verified':False, 'hardware_authorized':False}
    try:
        import dynamixel_sdk as sdk
        print('READ ONLY: no torque/goal/settings writes. Keep hand still; no other motor programs.', flush=True)
        collect(sdk,report)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped', error=str(exc) or 'Interrupted')
        print('STOP: '+report['error'], flush=True)
        print('If torque state is uncertain, disconnect motor power.', flush=True)
    finally:
        path = ROOT / 'hand12_readonly_diag_result.json'
        path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(f'Saved: {path}', flush=True)
    return 0 if report['status'] == 'read_complete_not_write_validation' else 1


if __name__ == '__main__':
    raise SystemExit(main())
