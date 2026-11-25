"""Fresh cube/palm images bracketed by encoder READs. No servo WRITE APIs.

Default is check-only. Only --execute opens the operator's camera/serial port.
The -7 profile is local to this capture; all motor launchers remain unchanged.
"""
import argparse
import datetime as dt
import functools
import hashlib
import json
from pathlib import Path
import time

from leap_camera_readonly_check import ReadonlyBus,validate_off

ROOT=Path(__file__).resolve().parent


def wait_for_capture_request(camera,poll,*,ready=None,now=time.monotonic,sleep=time.sleep):
    """Latch one SPACE; optionally wait for three fresh decoded marker pairs.

This is only a read-only photo visibility check. It never enables a motor.
"""
    requested=None;last_fresh=now();last_readable=None;streak=0
    while True:
        key=poll();packet=camera.read_packet();current=now()
        if key==' ' and requested is None:requested=current
        fresh=packet is not None and 0<=current-packet[0]<=.3
        if fresh:
            last_fresh=current
            if ready is None:streak=3
            elif ready(packet):
                streak=streak+1 if last_readable is not None and current-last_readable<=.6 else 1
                last_readable=current
            else:streak=0;last_readable=None
        if requested is not None:
            if fresh and streak>=3:return
            if current-max(last_fresh,requested)>=1.:
                raise RuntimeError('SPACE received but no fresh camera frame within 1s')
            if current-requested>=30.:
                raise RuntimeError('Palm ID40 + cube marker not readable together in three fresh frames within 30s; no paired capture started')
        sleep(.01)


def collect_pairs(read_snapshot,read_frame,save_frame,report,*,cancel,
                  count=12,now=time.monotonic,sleep=time.sleep):
    """Keep image analysis and encoding OUTSIDE the before/photo/after interval.

Timestamps are driver-return timestamps, not exposure timestamps. Stable
endpoints do not prove that the hand stayed still between those endpoints.
"""
    if not 1<=count<=12:raise ValueError('Capture count must be 1..12')
    rows=report.setdefault('pairs',[]);began=now()

    def snapshot(row,name):
        cancel();start=now();state=read_snapshot();end=now()
        row[name]=dict(read_started_monotonic=start,read_ended_monotonic=end,
                       read_ms=(end-start)*1000,snapshot=state)
        validate_off(state)
        if end-start>.2:raise RuntimeError('Fresh encoder READ exceeded 200ms; no writes sent')
        return state,end

    for n in range(count):
        if now()-began>20:raise RuntimeError('Finite capture exceeded 20s; partial pairs retained')
        row=dict(index=n,status='started');rows.append(row)
        before,ended=snapshot(row,'before')
        deadline=now()+1.
        while True:
            cancel();packet=read_frame();received=now()
            if packet is not None:
                stamp,frame=packet
                if ended<stamp<=received and received-stamp<=.3:break
            if received>=deadline:raise RuntimeError('No fresh frame after the before-READ; partial pairs retained')
            sleep(.005)
        row.update(camera_read_monotonic=stamp,frame_received_monotonic=received,
            image='pair_%02d.png'%n,
            timestamp_semantics='Camera timestamp is driver-read return, not sensor exposure')
        # READ immediately, before image encoding/detection or any GUI work.
        after,_=snapshot(row,'after')
        drift=[int(b)-int(a) for a,b in zip(before['position_ticks'],after['position_ticks'])]
        row.update(position_drift_ticks=drift,maximum_drift_ticks=max(map(abs,drift)),
                   stationary_endpoints=max(map(abs,drift))<=3,
                   continuous_stationarity_verified=False,
                   frame_bracket_ms=(row['after']['read_ended_monotonic']-
                                     row['before']['read_started_monotonic'])*1000)
        # Save raw images even when movement is observed. Never reuse old q.
        save_frame(row['image'],frame)
        row['status']='paired' if row['stationary_endpoints'] else 'endpoint_drift'
        sleep(.03)
    report['capture_elapsed_seconds']=now()-began


def analyze_pairs(folder,report):
    """Decode only this frame's markers; never turn the capture into a policy."""
    import cv2 as cv
    import numpy as np
    from marker_detector import MarkerDetector
    detector=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),
        range(6),hand_id=40,required_cube_count=1)
    good=[]
    for row in report.get('pairs',[]):
        path=folder/row.get('image','not_saved')
        if not path.is_file():continue
        blob=path.read_bytes()
        image=cv.imdecode(np.frombuffer(blob,np.uint8),cv.IMREAD_COLOR)
        if image is None:raise RuntimeError('Saved image could not be decoded: '+path.name)
        cs,ids,_=detector.detectMarkers(cv.cvtColor(image,cv.COLOR_BGR2GRAY))
        found=[] if ids is None else list(map(int,ids.ravel()))
        edges=detector.stats['min_edge_px_by_id']
        cube=[i for i in found if i in range(6)]
        size_ok=40 in found and edges.get('40',0)>=24 and any(edges.get(str(i),0)>=24 for i in cube)
        row['markers']=dict(ids=found,corners={str(i):np.asarray(c).reshape(4,2).tolist()
            for i,c in zip(found,cs)},minimum_edge_px=edges,detector_stats=dict(detector.stats),
            palm_and_cube_same_frame=bool(40 in found and cube),size_threshold_passed=bool(size_ok))
        row['image_sha256']=hashlib.sha256(blob).hexdigest()
        row['usable_paired_observation']=bool(row.get('status')=='paired' and size_ok)
        if row['usable_paired_observation']:good.append(row['index'])
    completed=[x for x in report.get('pairs',[]) if 'before' in x and 'after' in x]
    report.update(usable_pair_indices=good,usable_pair_count=len(good),
        minimum_usable_pairs=3,geometry_calibration_verified=False,
        old_encoder_readings_reused=False,
        completed_encoder_pair_count=len(completed),
        all_completed_pairs_have_off_endpoints=bool(completed) and all(
            not any(x[k]['snapshot']['torque']) for x in completed for k in ('before','after')))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--port',default='COM7')
    args=parser.parse_args(argv)
    print('READ ONLY CUBE POSE: fixed -7 camera exposure + fresh before/after encoders.',flush=True)
    print('No movement, targets, torque, gain, mode or watchdog-setting writes. No SAC.',flush=True)
    if not args.execute:
        print('CHECK ONLY: no camera, serial port, or settings opened.');return 0
    folder=ROOT/('cube_pose_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='started',
        servo_write_commands_sent=0,torque_changes_requested=0,policy_actions_sent=0,
        geometry_calibration_verified=False,old_encoder_readings_reused=False,
        requested_camera_exposure=-7.,auto_mode_meaning_verified=False,
        dimensions_mm=dict(cube=55.,palm_marker40=28.2,cube_marker5=10.7),
        motor_id_order=list(range(1,17)),sim_to_motor_ids=[8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1],
        mapping='all_finger_orders_user_confirmed_20261018_v2')
    camera=bus=None;restoration_path=folder/'camera_worker_result.json'
    try:
        from hand_readonly_check import require_serial_port
        from camera_stream import CameraStream
        from fixed_exposure_camera import fixed_exposure_worker
        import dynamixel_sdk as sdk
        import msvcrt
        import cv2 as cv
        require_serial_port(args.port,report)
        print('Leave current cube/hand as they are. USB + motor power connected; torque must already be OFF.',flush=True)
        print('Close other motor programs. Show palm ID40 and the cube top ID5 together as in the last photo.',flush=True)
        print('LIVE preview outlines decoded markers. Shade glare on ID40 until the GREEN READY line appears.',flush=True)
        print('SPACE once in THIS TERMINAL requests 12 pairs; it waits up to 30s for readable markers. Q exits.',flush=True)
        camera=CameraStream(report,worker=functools.partial(fixed_exposure_worker,
            restoration_path=str(restoration_path)),preview_title='LEAP cube pose READ ONLY')
        report['camera_metadata']=camera.meta

        def poll():
            camera.check_preview('READ ONLY: palm ID40 + cube ID5; SPACE in terminal; Q exits')
            if msvcrt.kbhit():
                key=msvcrt.getwch()
                if key in ('q','Q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
                return key
        from cube_marker_readiness import make_detector,inspect_frame
        detector=make_detector();last_notice=None
        report['preflight_marker_readiness']=dict(frames_examined=0,required_consecutive_frames=3)
        def readable(packet):
            nonlocal last_notice
            quality=inspect_frame(packet[1],detector)
            report['preflight_marker_readiness'].update(last_quality=quality,
                frames_examined=report['preflight_marker_readiness']['frames_examined']+1)
            notice=quality['instruction']
            if notice!=last_notice:print(notice,flush=True);last_notice=notice
            return quality['readable_pair']
        wait_for_capture_request(camera,poll,ready=readable)
        report['phase']='motor_ping'
        bus=ReadonlyBus(sdk,args.port);bus.open()
        report['motor_connection_verified']=True

        def save(name,frame):
            ok,encoded=cv.imencode('.png',frame)
            if not ok:raise RuntimeError('Cannot encode raw capture')
            (folder/name).write_bytes(encoded.tobytes())
        print('Recording 12 pairs now; keep hand, cube and camera still. No motor will be enabled.',flush=True)
        report['phase']='paired_capture'
        collect_pairs(bus.snapshot,camera.read_packet,save,report,cancel=poll)
        report['status']='capture_complete_pending_analysis'
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted',failed_phase=report.get('phase','preview'))
    finally:
        if bus is not None:
            try:bus.close()
            except Exception as exc:report.update(status='stopped',serial_cleanup_error=str(exc))
        if camera is not None:
            try:camera.release()
            except Exception as exc:report.update(status='stopped',camera_cleanup_error=str(exc))
        if restoration_path.is_file():
            try:report['camera_worker']=json.loads(restoration_path.read_bytes())
            except Exception as exc:report['camera_restore_report_error']=str(exc)
        report['camera_restoration_verified']=bool(report.get('camera_worker',{}).get(
            'restoration',{}).get('restored_original_readbacks'))
        try:analyze_pairs(folder,report)
        except Exception as exc:report.update(status='stopped',analysis_error=str(exc))
        if report['status']=='capture_complete_pending_analysis':
            if not report['camera_restoration_verified']:
                report.update(status='capture_saved_camera_restoration_unverified')
            elif report.get('usable_pair_count',0)>=3:
                report['status']='readonly_cube_pose_pairs_saved'
            else:report['status']='capture_saved_insufficient_paired_markers'
        payload=json.dumps(report,ensure_ascii=False,indent=2)
        (folder/'result.json').write_text(payload,encoding='utf-8')
        (ROOT/'leap_cube_pose_readonly_result.json').write_text(payload,encoding='utf-8')
        print('Saved:',folder,flush=True)
        print('Usable fresh photo/encoder pairs:',report.get('usable_pair_count',0),'/12; original camera readbacks restored:',report['camera_restoration_verified'],flush=True)
        print('No motor commands or torque changes sent. No rotation was attempted.',flush=True)
        if report['status']=='stopped':
            print('STOP:',report.get('error',report.get('analysis_error','See result')),flush=True)
            if report.get('failed_phase')=='motor_ping':
                print('Motor connection check failed before paired capture. Check separate motor power and adapter-to-hand cable; USB alone is not enough.',flush=True)
        else:print('Capture finished. Do not repeat; send just "done" for saved-file review.',flush=True)
    return 0 if report['status']=='readonly_cube_pose_pairs_saved' else 1


if __name__=='__main__':raise SystemExit(main())
