"""PING/READ only; preserve actual angles and stored goals as distinct observations."""
from pathlib import Path
import argparse,datetime as dt,json,time
import numpy as np
import leap_measured_grip_hold_20261117 as pose


class ReadOnlyBus(pose.PoseBus):
    def write(self,*args,**kwargs):raise RuntimeError('Read-only observation: motor writes prohibited')
    def off(self):raise RuntimeError('Read-only observation: torque changes prohibited')


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--record',action='store_true')
    ap.add_argument('--camera',action='store_true')
    ap.add_argument('--stage',choices=('middle_clear','cube_pushed','thumb_wrist_clear','index_middle_lower_seat','horizontal_thumbward_transfer'))
    args=ap.parse_args()
    if not args.record:print('CHECK ONLY: no devices opened.');return 0
    folder=pose.ROOT/('receiving_waypoint_readonly_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir();report=dict(kind='current_pose_readonly_observation',status='started',snapshots=[],motor_writes=0,
        torque_changes=0,fixed_grip_reference_changed=False,cube_support_observed=False,executable_waypoint=False)
    report['transfer_stage']=args.stage
    bus=None
    try:
        pose.validate_sources();_,fixed_actual,fixed_goal=pose.reference()
        bus=ReadOnlyBus(report,lambda:None)
        models=[]
        for motor in range(1,17):
            model,comm,error=bus.packet.ping(bus.port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} PING/model mismatch')
            models.append(int(model))
        report['models']=models
        for _ in range(2):
            sample=bus.snapshot()
            report['snapshots'].append({k:v.tolist() for k,v in sample.items()})
            time.sleep(.1)
        a,b=report['snapshots'];actual=np.array(b['position']);goal=np.array(b['goal'])
        torques=np.array([a['torque'],b['torque']])
        report['observed_torque_state']='OFF' if not np.any(torques) else ('ON' if np.all(torques==1) else 'mixed')
        report['manual_adjustment_allowed_by_observed_torque']=not bool(np.any(torques))
        report.update(status='readonly_observation_complete',max_two_read_difference_ticks=int(max(abs(actual-np.array(a['position'])))),
            actual_minus_fixed_measured_ticks=(actual-fixed_actual).tolist(),stored_goal_minus_fixed_goal_ticks=(goal-fixed_goal).tolist(),
            actual_angles_are_not_loaded_holding_goals=True)
    except Exception as exc:report.update(status='stopped',error=str(exc))
    finally:
        if bus is not None:bus.close()
        report['serial_port_closed']=True
        if args.camera and report.get('status')=='readonly_observation_complete':
            camera=None
            try:
                import cv2 as cv
                from camera_stream import CameraStream
                camera=CameraStream(report,timeout=10.)
                began=time.monotonic();packet=None
                while time.monotonic()-began<3.:
                    candidate=camera.read_packet()
                    if candidate and time.monotonic()-began>=.7 and 0<=time.monotonic()-candidate[0]<.3:
                        packet=candidate;break
                    time.sleep(.02)
                if packet is None:raise RuntimeError('No fresh manual-pose camera frame')
                okay,encoded=cv.imencode('.png',packet[1])
                if not okay:raise RuntimeError('PNG encoding failed')
                (folder/'current_pose.png').write_bytes(encoded.tobytes())
                report['camera_image']='current_pose.png'
                report['camera_frame_monotonic']=packet[0]
                report['camera_motor_hardware_synchronized']=False
            except Exception as exc:report['camera_error']=str(exc)
            finally:
                if camera is not None:camera.release()
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=str(folder),**{k:v for k,v in report.items() if k not in ('snapshots','models')}),ensure_ascii=False))
    if report['snapshots']:
        s=report['snapshots'][-1];print(json.dumps({k:s[k] for k in ('position','goal','torque','error')},ensure_ascii=False))
    return 0 if report['status']=='readonly_observation_complete' else 1


if __name__=='__main__':raise SystemExit(main())
