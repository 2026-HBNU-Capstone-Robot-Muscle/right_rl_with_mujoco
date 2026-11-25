"""Camera + native-screen timing check, with no motor interfaces or motion.

Replays the user's saved stable OFF request. It never opens a serial port and
cannot send motor goals. Default mode opens no device or simulator.
"""
import argparse
import datetime as dt
import json
import hashlib
from pathlib import Path
import time
from marker_trial_camera import MarkerTrialCamera
from book_empty_preparation_gate import run_screen,validate_proof
from book_empty_preparation_plan import BookPreparationPlan

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'book_prepare_empty_20261024_164116_703445/fresh_off_request.json'


class Camera(MarkerTrialCamera):
    EXPOSURE=-5.
    EMPTY_ONLY=True


def main():
    p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');args=p.parse_args()
    if not args.execute:
        print('CHECK ONLY: no camera, simulator, or motor interface opened.');return 0
    folder=ROOT/('book_screen_camera_validation_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(status='not_started',phase='screen_current_pose',recording_requested=True,
        motor_interfaces_opened=False,motor_commands_sent=0,screen_timeout_seconds=120)
    camera=None;began=time.monotonic()
    try:
        # Fork a clearly labelled offline request when the planner version has
        # changed. Motor readings stay unchanged and retain source provenance.
        data=SOURCE.read_bytes();payload=json.loads(data)
        if payload['plan_version']!=BookPreparationPlan.PLAN_VERSION:
            payload.update(plan_version=BookPreparationPlan.PLAN_VERSION,
                origin='offline_replay_for_new_plan_version',observation_source_sha256=hashlib.sha256(data).hexdigest(),
                hardware_execution_allowed=False)
            data=json.dumps(payload,indent=2).encode('utf-8')
        request=folder/'saved_off_request.json';request.write_bytes(data)
        plan=BookPreparationPlan(request)
        camera=Camera(folder/'camera',report)
        start=time.monotonic()
        proof=run_screen(request,folder/'screen.json',cancel=camera.check)
        report['native_screen_seconds']=time.monotonic()-start
        r=validate_proof(plan,proof)
        report.update(status='camera_and_full_native_screen_passed',proof_file=str(proof),
            static_cases=len(r['static_screens']),dynamic_cases=len(r['dynamic_sensitivity']),
            native_worker_elapsed_seconds=r.get('elapsed_seconds'))
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted')
    finally:
        if camera is not None:
            try:report['camera_recording']=camera.finish()
            except Exception as exc:report['camera_finish_error']=str(exc)
        report['total_seconds']=time.monotonic()-began
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        (ROOT/'book_screen_camera_validation_latest.json').write_text(json.dumps(dict(folder=str(folder)),ensure_ascii=False),encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k not in ('empty_start_marker_screen','camera_startup')},ensure_ascii=False),flush=True)
    return 0 if report['status']=='camera_and_full_native_screen_passed' else 1


if __name__=='__main__':raise SystemExit(main())
