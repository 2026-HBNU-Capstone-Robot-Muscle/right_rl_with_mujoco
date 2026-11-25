"""Bind fresh read-only telemetry and camera evidence; never open devices."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
from observed_book_model_policy import healthy_off
from hand_motor_mapping import SIM_TO_MOTOR_IDS,ticks_to_sim
R=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    sys.stdout.reconfigure(encoding='utf-8');ap=argparse.ArgumentParser();ap.add_argument('motor_folder');ap.add_argument('camera_folder');a=ap.parse_args();motor=R/a.motor_folder/'result.json';camera=R/a.camera_folder/'review.json';d=json.loads(motor.read_bytes());cam=json.loads(camera.read_bytes());assert d['status']=='readonly_data_saved' and d['servo_writes']==d['torque_changes']==0 and len(d['snapshots'])==3 and d['all_torque_off_read'] and d['maximum_drift_ticks']<=2
    for state in d['snapshots']:healthy_off(state)
    assert cam['status']=='frames_saved' and not cam['motor_api_used'] and len(cam['frames'])==3
    for f in cam['frames']:assert sha(camera.parent/f['file'])==f['sha256']
    s=d['snapshots'][-1];positions=np.asarray(s['position']);similar=[]
    for p in sorted(R.glob('resume_off_readonly_20261104_*/result.json')):
        prior=json.loads(p.read_bytes())
        if prior.get('snapshots'):
            delta=positions-np.asarray(prior['snapshots'][-1]['position']);similar.append(dict(source=p.relative_to(R).as_posix(),maximum_change_ticks=int(max(abs(delta))),changes_over16=[dict(motor_id=i+1,delta_ticks=int(x)) for i,x in enumerate(delta) if abs(x)>16]))
    out=R/'right_hand_rotation_20261106/connected_1728';out.mkdir(parents=True,exist_ok=True)
    request=dict(plan_version='yaw_current_off_recovery_diagnostic_20261027',motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),off_state=s,source=motor.relative_to(R).as_posix(),source_sha256=sha(motor),hardware_execution_allowed=False);(out/'off_request.json').write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    summary=dict(status='fresh_readonly_connected_state_reviewed',motor_source=motor.relative_to(R).as_posix(),motor_source_sha256=sha(motor),camera_source=camera.relative_to(R).as_posix(),camera_source_sha256=sha(camera),motor_count=16,baud=d['baud'],all_torque_off=True,all_hardware_errors_clear=not any(s['error']),all_watchdogs_clear=not any(s['watchdog']),maximum_drift_ticks=d['maximum_drift_ticks'],voltage_range_V=[min(s['voltage'])/10,max(s['voltage'])/10],temperature_range_C=[min(s['temperature']),max(s['temperature'])],position_ticks_by_id=positions.tolist(),joint_angles_sim_order_rad=ticks_to_sim(positions).tolist(),camera_device=cam['camera_startup']['device_name'],camera_frames=cam['frames'],older_pose_comparisons=similar,servo_writes=0,torque_changes=0,physical_cube_rotation_not_tested=True,new_motion_not_authorized_by_readonly_review=True);(out/'review.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8');print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
