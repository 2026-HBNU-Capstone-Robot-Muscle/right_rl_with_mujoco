"""Finish verification after a completed baud change; PING/READ only."""
import argparse,datetime as dt,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    p=argparse.ArgumentParser();p.add_argument('--read',action='store_true');args=p.parse_args()
    if not args.read:print('CHECK ONLY. --read opens existing1Mbps for PING/READ only.');return
    import dynamixel_sdk as sdk
    from hand_link_commission_1m_20261028 import Link,stress,save
    prior_path=ROOT/'post_commission_readonly_20261028_041740_872756/result.json'
    prior=json.loads(prior_path.read_bytes())
    assert prior['status']=='all16_1m_off_reads_saved' and not prior['differences'] and prior['maximum_drift_ticks']==0
    out=ROOT/('link_1m_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',baud=1000000,motor_writes=0,torque_changes=0,
        original_setup='link_commission_1m_20261028_041604_405637/result.json',
        independent_confirmation=prior_path.relative_to(ROOT).as_posix())
    link=Link(sdk,report);opened=False
    try:
        if not link.port.openPort():raise RuntimeError('Cannot open COM7')
        opened=True;link.rate(1000000)
        for m in range(1,17):
            model,comm,error=link.packet.ping(link.port,m)
            if comm or error or model!=1240:raise RuntimeError(f'ID{m} PING failed')
            if link.state(m)['baud_value']!=3:raise RuntimeError('Not all1Mbps')
        stress(link,report);report['status']='readonly_1m_camera_check_complete'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc) or 'Interrupted')
    finally:
        if opened:link.port.closePort()
        save(out/'result.json',report)
    print(json.dumps(dict(folder=out.name,status=report['status'],error=report.get('error'),
        readonly=report.get('readonly_check'),motor_writes=0)),flush=True)
if __name__=='__main__':main()
