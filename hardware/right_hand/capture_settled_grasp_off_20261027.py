"""PING/READ only. Preserve repeated OFF readings after the completed grasp pose."""
import contextlib,datetime,hashlib,io,json
from pathlib import Path
import numpy as np,dynamixel_sdk as sdk
from hand_readonly_check import require_serial_port,inspect_bus
ROOT=Path(__file__).resolve().parent
def main():
    folder=ROOT/'right_hand_rotation_20261027'/('grasp_off_'+datetime.datetime.now().strftime('%H%M%S'));folder.mkdir(parents=True);source=ROOT/'hand_readonly_check_result.json';first=json.loads(source.read_bytes());(folder/'original_readonly_result.json').write_bytes(source.read_bytes());snapshots=[first];log=io.StringIO()
    for _ in range(2):
        r=dict(timestamp=datetime.datetime.now().astimezone().isoformat(),status='not_started',motors=[],port='COM7',servo_write_commands_sent=0,torque_changes_requested=0)
        with contextlib.redirect_stdout(log):require_serial_port('COM7',r);inspect_bus(sdk,'COM7',r)
        snapshots.append(r)
    for r in snapshots:
        assert len(r['motors'])==16 and all(not m['torque_enabled'] and not m['hardware_error'] and not m['bus_watchdog_raw'] for m in r['motors'])
    positions=np.array([[m['position_ticks_signed'] for m in r['motors']] for r in snapshots]);result=dict(status='settled_grasp_off_readings_only',snapshots=snapshots,position_ticks=positions[-1].tolist(),drift_ticks=np.ptp(positions,axis=0).tolist(),maximum_drift_ticks=int(np.max(np.ptp(positions,axis=0))),motor_writes=0,torque_changes=0,cube_removed_user_confirmed=True,previous_completed_trial='fresh_empty_grasp_20261027_032949_661470/result.json',hardware_motion_allowed=False)
    (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');(folder/'read_log.txt').write_text(log.getvalue(),encoding='utf-8');(ROOT/'right_hand_rotation_20261027/grasp_off_latest.json').write_text(json.dumps(dict(folder=folder.relative_to(ROOT).as_posix(),result=(folder/'result.json').relative_to(ROOT).as_posix(),sha256=hashlib.sha256((folder/'result.json').read_bytes()).hexdigest()),indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in result.items() if k!='snapshots'},indent=2))
if __name__=='__main__':main()
