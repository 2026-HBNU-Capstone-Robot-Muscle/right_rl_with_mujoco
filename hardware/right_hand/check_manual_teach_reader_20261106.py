"""Test the new OFF teaching reader on the existing link, PING/READ only."""
import argparse,datetime as dt,json
from pathlib import Path
from manual_grasp_off_teach_20261106 import TeachingReadOnlyBus,validate_off_snapshot
R=Path(__file__).resolve().parent
def main():
    p=argparse.ArgumentParser();p.add_argument('--read',action='store_true');a=p.parse_args()
    if not a.read:print('CHECK ONLY. No device opened.');return
    report=dict(status='started',servo_writes=0,torque_changes=0,snapshots=[]);bus=TeachingReadOnlyBus(report,lambda:None)
    try:
        bus.open()
        for _ in range(2):
            state=bus.snapshot();validate_off_snapshot(state);report['snapshots'].append({k:v.tolist() for k,v in state.items()})
        report['status']='OFF_teaching_reader_verified'
    except Exception as exc:report.update(status='stopped',error=str(exc))
    finally:bus.close()
    out=R/'right_hand_rotation_20261106/manual_grasp'/('reader_check_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S')+'.json');out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(source=out.relative_to(R).as_posix(),status=report['status'],error=report.get('error'),instructions=report.get('instruction_counts'),snapshots=len(report['snapshots']),servo_writes=0,torque_changes=0)))
if __name__=='__main__':main()
