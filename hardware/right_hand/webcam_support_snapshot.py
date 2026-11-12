"""Bounded, named-camera snapshots for support inspection. No motor modules."""
import argparse
from datetime import datetime,timezone,timedelta
import hashlib
import json
from pathlib import Path
import time

ROOT=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true',help='Open only the expected webcam and save three fresh frames.')
    args=parser.parse_args()
    if not args.execute:
        print('Camera stays closed. Use --execute for an authorized camera-only snapshot.')
        return 0
    import cv2 as cv
    from camera_stream import CameraStream
    folder=ROOT/f'webcam_support_view_20261024_{datetime.now().strftime("%H%M%S_%f")}'
    folder.mkdir()
    report=dict(status='starting',origin='user_requested_webcam_inspection',
        client_date='2026-10-24',motors_accessed=False,motor_commands_sent=0,
        camera_property_changes_requested=False,frames=[])
    stream=None
    try:
        stream=CameraStream(report,timeout=10.)
        report['camera_metadata']=stream.meta
        began=time.monotonic();last_saved=began;warmup=0
        while len(report['frames'])<3:
            if time.monotonic()-began>8:
                raise RuntimeError('No three fresh camera frames within 8 seconds.')
            packet=stream.read_packet()
            if packet is None:
                time.sleep(.02);continue
            stamp,frame=packet
            age=time.monotonic()-stamp
            if not 0<=age<=.3:continue
            warmup+=1
            if warmup<10 or time.monotonic()-last_saved<.3:continue
            if frame.shape!=(720,1280,3):raise RuntimeError('Unexpected frame size.')
            success,encoded=cv.imencode('.png',frame)
            if not success:raise RuntimeError('Frame encoding failed.')
            name=f'frame_{len(report["frames"])+1}.png';blob=encoded.tobytes()
            (folder/name).write_bytes(blob)
            report['frames'].append(dict(file=name,driver_read_monotonic=stamp,
                age_at_capture_seconds=age,sha256=hashlib.sha256(blob).hexdigest(),
                mean_intensity=float(frame.mean()),standard_deviation=float(frame.std())))
            last_saved=time.monotonic()
        report['status']='fresh_camera_snapshots_saved'
    except Exception as exc:
        report.update(status='stopped',error=f'{type(exc).__name__}: {exc}')
    finally:
        if stream is not None:stream.release()
        report['capture_wall_clock_iso']=datetime.now(timezone(timedelta(hours=9))).isoformat()
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        (ROOT/'webcam_support_snapshot_latest.json').write_text(json.dumps(dict(folder=str(folder),status=report['status']),ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(folder=str(folder),status=report['status'],error=report.get('error'),
        camera_worker_stopped=report.get('camera_worker_stopped'),frames=len(report['frames']),
        motors_accessed=False,motor_commands_sent=0),ensure_ascii=False,indent=2),flush=True)
    return 0 if report['status']=='fresh_camera_snapshots_saved' else 1


if __name__=='__main__':
    raise SystemExit(main())
