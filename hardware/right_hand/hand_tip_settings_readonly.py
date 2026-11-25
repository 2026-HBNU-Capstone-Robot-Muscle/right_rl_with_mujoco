"""PING/READ only: compare index/middle/ring fingertip motors (IDs 8,12,16).

No motion-driver imports, SDK writes, reboot, resets or automatic corrections.
Register addresses: https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/
Differences are observations, not proof of a fault or an instruction to match.
"""
import datetime
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
COMPARE_IDS = (8,12,16)
FIELDS = (
    ('firmware',6,1),('drive_mode',10,1),('operating_mode',11,1),
    ('homing_offset_raw',20,4),('pwm_limit_raw',36,2),('current_limit_raw',38,2),
    ('max_position_limit_raw',48,4),('min_position_limit_raw',52,4),
    ('startup_configuration',60,1),('pwm_slope_raw',62,1),('shutdown_mask',63,1),
    ('velocity_i',76,2),('velocity_p',78,2),('position_d',80,2),('position_i',82,2),
    ('position_p',84,2),('feedforward_2nd',88,2),('feedforward_1st',90,2),
    ('watchdog',98,1),('goal_pwm_raw',100,2),('goal_current_raw',102,2),
    ('profile_acceleration',108,4),('profile_velocity',112,4))


def collect(sdk,report):
    port,packet = sdk.PortHandler('COM7'),sdk.PacketHandler(2.0)
    report.update(read_events=[],configurations={},all_torque_off_verified=False)
    def read(motor,address,size=1):
        fn = {1:packet.read1ByteTxRx,2:packet.read2ByteTxRx,4:packet.read4ByteTxRx}[size]
        value,comm,error = fn(port,motor,address)
        report['read_events'].append({'id':motor,'address':address,'size':size,
                                     'value':int(value),'comm':int(comm),'device_error':int(error)})
        if comm!=sdk.COMM_SUCCESS or error:
            raise RuntimeError(f'ID{motor} read {address}: comm={comm}, device_error={error}')
        return int(value)
    def off(motor):
        torque=read(motor,64)
        if torque!=0: raise RuntimeError(f'ID{motor} torque ON: cut motor power; no automatic OFF write')
        if read(motor,70)!=0: raise RuntimeError(f'ID{motor} hardware error; no automatic reset')
        return torque
    try:
        if not port.openPort() or not port.setBaudRate(3000000):
            raise RuntimeError('Cannot open COM7 at 3Mbps; close other motor programs')
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if model!=1240 or comm!=sdk.COMM_SUCCESS or error:
                raise RuntimeError(f'ID{motor}: model/communication mismatch; no control-table interpretation')
            off(motor)
        report['dynamic_state']={}
        for motor in COMPARE_IDS:
            off(motor)
            config={name:read(motor,addr,size) for name,addr,size in FIELDS}
            report['configurations'][str(motor)]=config
            report['dynamic_state'][str(motor)]={name:read(motor,addr,size) for name,addr,size in
                [('goal_raw',116,4),('present_raw',132,4),('present_current_raw',126,2),
                 ('voltage_raw',144,2),('temperature_c',146,1)]}
            print(f'ID{motor}: mode={config["operating_mode"]}, P/I/D='
                  f'{config["position_p"]}/{config["position_i"]}/{config["position_d"]}, '
                  f'current_limit={config["current_limit_raw"]}, goal_current={config["goal_current_raw"]}, '
                  f'pwm_limit={config["pwm_limit_raw"]}, torque OFF',flush=True)
        # A second complete configuration read detects changes by other software.
        for motor in COMPARE_IDS:
            off(motor)
            first=report['configurations'][str(motor)]
            for name,addr,size in FIELDS:
                value=read(motor,addr,size)
                if value!=first[name]:
                    raise RuntimeError(f'ID{motor} configuration changed during read: {name}')
        report['differences']={}
        for name,_,_ in FIELDS:
            values={str(i):report['configurations'][str(i)][name] for i in COMPARE_IDS}
            if len(set(values.values()))>1:
                report['differences'][name]=values
                print(f'DIFFERENCE {name}: {values}',flush=True)
        report['final_torque_states']={str(i):off(i) for i in range(1,17)}
        report['all_torque_off_verified']=True
        report['status']='read_complete_not_motion_approval'
        if not report['differences']: print('Selected configuration fields match across IDs 8, 12 and 16.',flush=True)
        print('Differences do not prove a fault. Mechanical load is NOT tested. No settings changed.',flush=True)
    finally:
        port.closePort()  # Host-side close only; no motor command.


def main():
    report={'timestamp':datetime.datetime.now().astimezone().isoformat(),
            'status':'not_started','servo_write_commands_sent':0,'policy_used':False}
    try:
        import dynamixel_sdk as sdk
        print('READ ONLY: compare IDs 8/12/16. PING/READ only; NO torque, goal, gain or EEPROM writes.',flush=True)
        print('Close all other motor programs. Keep fingers clear. No movement is requested.',flush=True)
        collect(sdk,report)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted')
        print('STOP: '+report['error'],flush=True)
        print('If torque state is uncertain, cut motor power.',flush=True)
    finally:
        tag=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        for name in (f'hand_tip_settings_readonly_{tag}.json','hand_tip_settings_readonly_result.json'):
            path=ROOT/name
            path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(f'Saved: {path}',flush=True)
    return 0 if report['status']=='read_complete_not_motion_approval' else 1


if __name__=='__main__': raise SystemExit(main())
