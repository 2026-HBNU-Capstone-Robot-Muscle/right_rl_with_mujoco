"""Camera-only preview/capture. No SDK, serial ports or motor settings."""
import datetime as dt
import json
from pathlib import Path
import time
from camera_frame_signal import CameraImageUnavailable

ROOT=Path(__file__).resolve().parent


def main():
    import msvcrt
    from empty_trial_camera import EmptyTrialCamera
    stamp=dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder=ROOT/f'camera_signal_check_{stamp}';folder.mkdir()
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='not_started',
                phase='camera_only_preview',recording_requested=False,
                hardware_commands_sent=0,motor_port_opened=False,
                camera_instruction='CAMERA ONLY: show whole hand; SPACE saves picture; Q exits')
    camera=None
    print('CAMERA ONLY. No motor port or motor movement.',flush=True)
    print('Show the whole robot hand in the preview. If black, check lens/connection/camera app.',flush=True)
    print('SPACE in this TERMINAL saves a fresh picture after the image recovers. Q exits.',flush=True)
    try:
        camera=EmptyTrialCamera(folder/'camera',report)
        while True:
            camera.check()
            if msvcrt.kbhit():
                key=msvcrt.getwch()
                if key in ('q','Q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
                if key==' ':
                    try:camera.require_fresh()
                    except CameraImageUnavailable as exc:print(str(exc),flush=True);continue
                    break
            time.sleep(.01)
        report['recording_requested']=True;began=time.monotonic()
        while True:
            camera.check()
            photo=camera.capture_named('preview.jpg',after=began+.2)
            if photo is not None:
                report.update(status='camera_image_signal_confirmed',photo=photo,
                              hand_view_not_automatically_verified=True)
                break
            if time.monotonic()-began>2:raise RuntimeError('No usable fresh camera picture')
            time.sleep(.02)
        print('Picture saved. No motor movement. Tell the assistant: 했어',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted');print('STOP:',report['error'],flush=True)
    finally:
        if camera is not None:report['camera_recording']=camera.finish()
        payload=json.dumps(report,ensure_ascii=False,indent=2)
        (folder/'result.json').write_text(payload,encoding='utf-8')
        (ROOT/'leap_camera_signal_check_result.json').write_text(payload,encoding='utf-8')
        print('Saved:',folder,flush=True)
    return 0 if report['status']=='camera_image_signal_confirmed' else 1


if __name__=='__main__':raise SystemExit(main())
