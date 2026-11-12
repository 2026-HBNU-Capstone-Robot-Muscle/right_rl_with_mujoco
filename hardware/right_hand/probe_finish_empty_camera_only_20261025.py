"""Exercise the FIRST SPACE camera path immediately, without any motor bus."""
import datetime,json,time
from pathlib import Path
from quarter_finish_empty_camera_20261025 import QuarterFinishEmptyCamera
ROOT=Path(__file__).resolve().parent
def main():
    folder=ROOT/('finish_empty_camera_only_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir();report=dict(timestamp=datetime.datetime.now().astimezone().isoformat(),phase='preview',hardware_authorized=False,recording_requested=False,status='not_started',motor_port_opened=False,motor_goal_writes=0,torque_changes=0);camera=None
    try:
        camera=QuarterFinishEmptyCamera(folder/'camera',report)
        # This is deliberately immediate, reproducing SPACE arriving before
        # the asynchronous recorder has received its first camera packet.
        report['first_packet_present_before_require']=camera.latest is not None
        camera.require_fresh();report['pending_first_space_accepted']=True;report['recording_requested']=True
        began=time.monotonic()
        while time.monotonic()-began<1.:camera.check();time.sleep(.03)
        report['photo']=camera.capture_named('camera_only_preview.jpg');report['status']='camera_only_preview_verified'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc) or 'Interrupted')
    finally:
        if camera is not None:report['camera_recording']=camera.finish()
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k not in ('camera_exposure_worker','camera_image_signal')},ensure_ascii=False),flush=True)
    return 0 if report['status']=='camera_only_preview_verified' and report['camera_recording']['status']=='complete' else 1
if __name__=='__main__':raise SystemExit(main())
