"""Preserve the freshly authorized OFF snapshot and repeat reads for stability."""
import contextlib,datetime,hashlib,io,json
from pathlib import Path
import numpy as np
import dynamixel_sdk as sdk
from hand_readonly_check import inspect_bus,require_serial_port
ROOT=Path(__file__).resolve().parent
def main():
    folder=ROOT/'right_hand_rotation_20261026'/('fresh_empty_start_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'));folder.mkdir();source=ROOT/'hand_readonly_check_result.json';first=json.loads(source.read_bytes());(folder/'original_readonly_result.json').write_bytes(source.read_bytes());snapshots=[first];log=io.StringIO()
    for _ in range(2):
        report=dict(status='not_started',timestamp=datetime.datetime.now().astimezone().isoformat(),motors=[],port='COM7',servo_write_commands_sent=0,torque_changes_requested=0)
        with contextlib.redirect_stdout(log):require_serial_port('COM7',report);inspect_bus(sdk,'COM7',report)
        snapshots.append(report)
    positions=np.asarray([[m['position_ticks_signed'] for m in r['motors']] for r in snapshots]);drift=np.ptp(positions,axis=0);latest=snapshots[-1];result=dict(status='fresh_torque_off_angles_saved',source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),snapshots=snapshots,positions_ticks=positions.tolist(),max_drift_ticks=int(drift.max()),drift_ticks=drift.tolist(),stable_endpoints=bool(drift.max()<=4),continuous_stationarity_verified=False,motor_writes_sent=0,torque_changes_sent=0,cube_removed_user_confirmed=True,position_ticks=positions[-1].tolist(),min_voltage_v=min(m['voltage_v'] for r in snapshots for m in r['motors']),max_temperature_c=max(m['temperature_c'] for r in snapshots for m in r['motors']),hardware_motion_allowed=False)
    (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');(folder/'read_log.txt').write_text(log.getvalue(),encoding='utf-8');handoff=ROOT/'right_hand_rotation_20261026/fresh_empty_start_latest.json';handoff.write_text(json.dumps(dict(folder=folder.relative_to(ROOT).as_posix(),result=(folder/'result.json').relative_to(ROOT).as_posix(),result_sha256=hashlib.sha256((folder/'result.json').read_bytes()).hexdigest(),position_ticks=result['position_ticks'],stable_endpoints=result['stable_endpoints'],max_drift_ticks=result['max_drift_ticks'],hardware_motion_allowed=False),indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(folder=str(folder),**{k:v for k,v in result.items() if k not in ('snapshots','positions_ticks')}),indent=2))
if __name__=='__main__':main()
