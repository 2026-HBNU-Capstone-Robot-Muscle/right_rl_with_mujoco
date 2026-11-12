"""Autonomous camera-only stability recording. No motor imports or port."""
import argparse
import datetime as dt
import json
from pathlib import Path
import time
from leap_book_cube_grasp_trial import Camera


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    if not args.execute:print('CHECK ONLY: no camera opened');return 0
    root=Path(__file__).resolve().parent
    folder=root/('book_camera_confirmation_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    r=dict(status='not_started',phase='camera_only_stability',recording_requested=True,
        motor_interface_opened=False,servo_write_commands_sent=0)
    camera=None
    try:
        camera=Camera(folder/'camera',r);began=time.monotonic()
        while time.monotonic()-began<65:
            camera.check();time.sleep(.02)
        r['status']='camera_stability_observed';r['duration_seconds']=time.monotonic()-began
    except (Exception,KeyboardInterrupt) as exc:r.update(status='stopped',error=str(exc))
    finally:
        if camera is not None:r['camera_recording']=camera.finish()
        (folder/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
        (root/'book_camera_confirmation_latest.json').write_text(json.dumps(dict(folder=str(folder)),indent=2),encoding='utf-8')
        print(json.dumps(dict(folder=str(folder),status=r['status'],error=r.get('error'),camera_recording=r.get('camera_recording')),ensure_ascii=False))
    return 0 if r['status']=='camera_stability_observed' and r['camera_recording']['status']=='complete' else 1


if __name__=='__main__':raise SystemExit(main())
