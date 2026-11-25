"""PING/READ at confirmed1Mbps; no calls to any motor write method."""
import argparse,datetime as dt,json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    p=argparse.ArgumentParser();p.add_argument('--read',action='store_true');args=p.parse_args()
    if not args.read:print('CHECK ONLY. --read uses PING/READ at1Mbps; no setting writes.');return
    import dynamixel_sdk as sdk
    from hand_link_commission_1m_20261028 import Link
    prior=json.loads((ROOT/'link_commission_1m_20261028_041604_405637/result.json').read_bytes())
    out=ROOT/('post_commission_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',baud=1000000,motor_writes=0,snapshots=[],individual_states={},differences=[])
    link=Link(sdk,report);opened=False
    try:
        if not link.port.openPort():raise RuntimeError('Cannot open COM7')
        opened=True;link.rate(1000000)
        for motor in range(1,17):
            model,comm,error=link.packet.ping(link.port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} PING failed at1Mbps')
            a=link.state(motor);b=link.state(motor)
            report['individual_states'][str(motor)]=[a,b]
            before=prior['before'][str(motor)]['state']
            for key,value in before.items():
                if key in ('baud_value','voltage','temperature'):continue
                if a[key]!=value or b[key]!=value:report['differences'].append(dict(motor_id=motor,address=int(key),before=value,after=[a[key],b[key]]))
        regs=dict(torque=(64,1),error=(70,1),d_gain=(80,2),i_gain=(82,2),p_gain=(84,2),watchdog=(98,1),
            current_cap=(102,2),acceleration=(108,4),velocity=(112,4),goal=(116,4),current=(126,2),position=(132,4),voltage=(144,2),temperature=(146,1))
        for _ in range(3):
            group=sdk.GroupSyncRead(link.port,link.packet,64,83)
            try:
                for m in range(1,17):group.addParam(m)
                comm=group.txPacket()
                if not comm:link.port.setPacketTimeoutMillis(80.);comm=group.rxPacket()
                if comm or not all(group.isAvailable(m,64,83) for m in range(1,17)):raise RuntimeError('Incomplete1Mbps OFF snapshot')
                state={key:[int(group.getData(m,a,n)) for m in range(1,17)] for key,(a,n) in regs.items()}
                if any(state['torque']) or any(state['error']) or any(state['watchdog']):raise RuntimeError('Not healthy OFF; no writes sent')
                report['snapshots'].append(state)
            finally:group.clearParam()
            time.sleep(.1)
        import numpy as np
        report['maximum_drift_ticks']=int(np.ptp([s['position'] for s in report['snapshots']],axis=0).max())
        report['status']='all16_1m_off_reads_saved'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc) or 'Interrupted')
    finally:
        if opened:link.port.closePort()
        (out/'result.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
    print(json.dumps(dict(folder=out.name,status=report['status'],error=report.get('error'),differences=report['differences'],
        drift=report.get('maximum_drift_ticks'),position=report['snapshots'][-1]['position'] if report['snapshots'] else None,motor_writes=0)))
if __name__=='__main__':main()
