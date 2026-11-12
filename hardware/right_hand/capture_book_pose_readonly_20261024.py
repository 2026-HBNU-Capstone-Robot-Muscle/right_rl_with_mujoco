"""One automatic, read-only photo/encoder pair after the book setup change.

No servo WRITE instruction, torque change, goal, mode, gain or EEPROM access.
Uses the same previously tested PING/READ inspection and pair acquisition.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    args=p.parse_args()
    if not args.execute:
        print('No devices opened. --execute permits only PING/READ plus a fresh camera frame.')
        return 0
    import cv2 as cv
    import dynamixel_sdk as sdk
    from hand_readonly_check import inspect_bus,require_serial_port
    from camera_stream import CameraStream
    from leap_pose_pair_readonly import capture_pair
    folder=ROOT/f'book_pose_readonly_20261024_{datetime.now().strftime("%H%M%S_%f")}'
    folder.mkdir()
    report=dict(status='starting',port='COM7',reference_label='current_book_support_side',
        origin='task_readonly_observation_after_reported_20mm_book_setup',
        servo_write_commands_sent=0,torque_changes_requested=0,policy_actions_sent=0,
        physical_calibration_verified=False,book_thickness_mm_user_reported=20.)
    camera=None
    try:
        require_serial_port('COM7',report)
        camera=CameraStream(report,timeout=10.)
        def inspect(row):
            require_serial_port('COM7',row)
            inspect_bus(sdk,'COM7',row)
        def save_image(frame):
            success,png=cv.imencode('.png',frame)
            if not success:raise RuntimeError('PNG encoding failed.')
            (folder/'paired_photo.png').write_bytes(png.tobytes())
        capture_pair(inspect,camera,report,save_image)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=f'{type(exc).__name__}: {exc}')
    finally:
        if camera is not None:camera.release()
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        (ROOT/'book_pose_readonly_latest.json').write_text(json.dumps(dict(folder=str(folder),status=report['status']),ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(folder=str(folder),status=report['status'],error=report.get('error'),
        snapshots=len(report.get('snapshots',[])),pair_check=report.get('pair_check'),
        servo_write_commands_sent=0,torque_changes_requested=0,camera_worker_stopped=report.get('camera_worker_stopped')),
        ensure_ascii=False,indent=2),flush=True)
    return 0 if report['status']=='photo_and_readings_saved' else 1


if __name__=='__main__':
    raise SystemExit(main())
