"""Compact saved diagnostic review and saved-endpoint path preparation. No devices."""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024'


def main():
    reports=[]
    for folder in sorted(ROOT.glob('bus_readonly_diagnose_20261024_*')):
        source=folder/'result.json';blob=source.read_bytes();r=json.loads(blob)
        scans=r.get('motor_scans',[]);samples=r.get('stream_samples',[])
        review=dict(source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(blob).hexdigest(),
            status=r['status'],error=r.get('error'),failed_phase=r.get('failed_phase'),
            hardware_response_check_passed=r['hardware_response_check_passed'],
            ping_attempts=len(scans),successful_ping_count=sum(s.get('model_verified',False) for s in scans),
            successful_health_checks=sum(s.get('health_complete',False) for s in scans),
            ping_success_count_by_id=r.get('ping_success_count_by_id'),
            stream_samples=len(samples),stream_complete=r.get('stream_complete'),
            failed_receive_packets=r.get('failed_receive_packets',[]),
            all_16_torque_OFF_verified=r.get('all_16_torque_OFF_verified'),
            servo_write_commands_sent=r['servo_write_commands_sent'],camera_opened=r['camera_opened'])
        if samples:
            review.update(stream_duration_last_t_s=samples[-1]['t'],
                minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
                maximum_temperature_c=max(max(s['temperature_c']) for s in samples),
                read_ms_quantiles=np.quantile([s['read_ms'] for s in samples],[0,.5,.95,1]).tolist(),
                final_position_ticks=samples[-1]['position_ticks'],
                maximum_position_range_ticks=int(np.max(np.ptp(np.array([s['position_ticks'] for s in samples]),axis=0))))
        reports.append(review)
        compact={k:v for k,v in review.items() if k!='failed_receive_packets'}
        compact['failed_receive_packet_count']=len(review['failed_receive_packets'])
        print(json.dumps(compact,ensure_ascii=False))
    latest=reports[-1]
    if not latest['hardware_response_check_passed']:
        raise RuntimeError('Latest diagnostic is not complete; no motion-path preparation')
    # These real OFF readings contain exactly the six fields used by the native
    # geometry checker. Do not fabricate drive settings or a goal readback.
    original=json.loads((ROOT/latest['source']).read_bytes());row=original['stream_samples'][-1]
    state=dict(position=np.array(row['position_ticks']),torque=np.array(row['torque']),
        error=np.array(row['hardware_error']),watchdog=np.array(row['watchdog']),
        voltage=np.rint(np.array(row['voltage_v'])*10).astype(int),temperature=np.array(row['temperature_c']))
    path_review=dict(source=latest['source'],source_sha256=latest['source_sha256'],
        saved_endpoint_not_current_motion_approval=True,real_devices_opened=False)
    try:
        from grasped_yaw_plan import GraspedYawPlan
        from thumb_live_start_gate import write_request
        folder=OUT/'grasped_yaw_reconnected_readonly_fixture';folder.mkdir(exist_ok=True)
        request=write_request(folder,state,GraspedYawPlan.PLAN_VERSION,GraspedYawPlan.validate_off_state)
        data=json.loads(request.read_bytes());data.update(origin='saved_read_only_OFF_endpoint_not_live_motion_approval',
            saved_source=latest['source'],saved_source_sha256=latest['source_sha256'])
        request.write_text(json.dumps(data,indent=2),encoding='utf-8')
        plan=GraspedYawPlan(request);raw=plan.source_position();plan.validate_start(raw)
        (folder/'request.json').write_bytes(request.read_bytes())
        path_review.update(planner_start_passed=True,goals=len(plan.plan(raw)),request=str(request.relative_to(ROOT)))
    except (ValueError,RuntimeError) as exc:
        path_review.update(planner_start_passed=False,error=str(exc))
    record=dict(diagnostics=reports,path_preparation=path_review,physical_rotation_verified=False,
        live_sac=False,real_devices_opened=False)
    (OUT/'all_id_readonly_results_review.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
    print('PATH_PREPARATION',json.dumps(path_review,ensure_ascii=False))


if __name__=='__main__':main()
