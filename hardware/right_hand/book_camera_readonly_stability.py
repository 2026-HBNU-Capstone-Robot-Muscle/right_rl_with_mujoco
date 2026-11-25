"""Bounded camera-only check of the existing -5 exposure worker. No motor imports."""
import argparse
import datetime as dt
import json
from pathlib import Path
import time
from marker_trial_camera import MarkerTrialCamera

ROOT=Path(__file__).resolve().parent


class Camera(MarkerTrialCamera):
    EXPOSURE=-5.
    EMPTY_ONLY=True


def main():
    p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');args=p.parse_args()
    if not args.execute:
        print('CHECK ONLY: no camera or motor interface opened.');return 0
    folder=ROOT/('book_camera_stability_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(status='not_started',phase='camera_only_stability',recording_requested=True,
        motor_commands_sent=0,motor_interfaces_opened=False,requested_seconds=12.)
    camera=None
    try:
        camera=Camera(folder/'camera',report);began=time.monotonic()
        while time.monotonic()-began<12.:
            camera.check();time.sleep(.02)
        report['status']='camera_profile_stable_observed'
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted')
    finally:
        if camera is not None:
            try:report['camera_recording']=camera.finish()
            except Exception as exc:report['camera_finish_error']=str(exc)
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(dict(folder=str(folder),status=report['status'],error=report.get('error'),
            camera_recording=report.get('camera_recording')),ensure_ascii=False),flush=True)
    return 0 if report['status']=='camera_profile_stable_observed' else 1


if __name__=='__main__':raise SystemExit(main())
