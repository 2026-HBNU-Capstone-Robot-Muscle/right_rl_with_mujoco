"""One fresh photo bracketed by read-only encoder snapshots. Never sends writes.

This tests consistency of photo/encoder evidence, not geometric calibration.
Camera timestamps are driver-read times, not synchronized exposure timestamps.
"""
import argparse
import datetime as dt
import json
from pathlib import Path
import time
import math

ROOT = Path(__file__).resolve().parent


def validate_pair(before, after, max_drift_ticks=3):
    positions = []
    for snapshot in (before, after):
        rows = snapshot.get('motors', [])
        if snapshot.get('status') != 'read_complete_not_motion_approval' or sorted(r['id'] for r in rows) != list(range(1,17)):
            raise RuntimeError('Incomplete read; photo/encoder pair is not accepted')
        for row in rows:
            if row.get('torque_enabled') != 0 or row.get('torque_enabled_end') != 0 or row.get('hardware_error') != 0:
                raise RuntimeError('Torque/error state invalid; no changes will be sent')
        positions.append({r['id']:r['position_ticks_signed'] for r in rows})
    drift = {str(i):abs(positions[1][i]-positions[0][i]) for i in range(1,17)}
    return dict(position_drift_ticks=drift, maximum_drift_ticks=max(drift.values()),
                stationary_endpoints=max(drift.values()) <= max_drift_ticks,
                threshold_ticks=max_drift_ticks,
                continuous_stationarity_verified=False, geometry_calibration_verified=False)


def summarize_tip_direction(pairs, intended_action='manually flex middle fingertip inward, with motor power OFF'):
    """Report changed IDs without assuming which encoder belongs to the fingertip."""
    if len(pairs)!=2:raise RuntimeError('Two stable photo/read pairs required')
    positions=[]
    for pair in pairs:
        if pair.get('status')!='photo_and_readings_saved':raise RuntimeError('Unaccepted direction pair')
        if not validate_pair(*pair['snapshots'])['stationary_endpoints']:
            raise RuntimeError('Finger moved during one direction capture')
        positions.append({i:sum(next(r['position_ticks_signed'] for r in s['motors'] if r['id']==i)
                                for s in pair['snapshots'])/2 for i in range(1,17)})
    delta={str(i):(positions[1][i]-positions[0][i])*360/4096 for i in range(1,17)}
    largest=max(range(1,17),key=lambda i:abs(delta[str(i)]))
    other=[i for i in range(1,17) if i!=largest and abs(delta[str(i)])>3]
    enough=abs(delta[str(largest)])>=5
    return dict(delta_degrees_by_motor=delta,largest_changed_motor_id=largest,
                largest_change_degrees=delta[str(largest)],
                other_motors_changed_over_3deg=other,
                status='ready_for_visual_direction_review' if enough and not other else 'multiple_changes_or_small_motion_needs_review',
                assumed_anatomical_motor_id=None,
                intended_action=intended_action,
                physical_direction_verified=False,calibration_selected=False)


def direction_protocol(reference):
    """Physical instructions stay separate from unverified motor-ID assignments."""
    if reference=='middle_tip_direction':
        return dict(action='manually flex middle fingertip inward, with motor power OFF',
                    overlay='POWER OFF: flex middle TIP only. Power ON, hands away, SECOND SPACE.',
                    instructions=['Gently bend ONLY the MIDDLE FINGERTIP toward the palm by about 10-20 degrees.',
                                  'Support its adjacent link so the root and middle joint stay still. Do not force or push into contact.'],
                    pointer='leap_middle_direction_readonly_result.json')
    if reference=='middle_pip_order':
        return dict(action='manually flex middle second hinge from fingertip; move distal motor housing and pad together, with motor power OFF',
                    overlay='POWER OFF: middle SECOND hinge from TIP. Move tip + its motor together.',
                    instructions=['Bend the MIDDLE finger at the SECOND bending hinge counting from the fingertip, about 10-15 degrees inward.',
                                  'Move the fingertip pad AND the motor housing directly below it TOGETHER.',
                                  'Keep the tip-to-housing angle unchanged; support the lower link so the root stays still.',
                                  'This is NOT the fingertip-only movement from the previous capture. Never force or push into contact.'],
                    pointer='leap_middle_joint_order_readonly_result.json')
    return None


def capture_pair(inspect, camera, report, save_image, *, now=time.monotonic, sleep=time.sleep,
                 burst_seconds=0, score_image=None, save_candidate=None):
    """Dependency-injected core. The only bus capability supplied is inspection."""
    report['snapshots'] = []
    def read():
        row = dict(status='started', timestamp=dt.datetime.now().astimezone().isoformat(),
                   motors=[], servo_write_commands_sent=0, torque_changes_requested=0)
        report['snapshots'].append(row)
        row['read_start_monotonic'] = now()
        inspect(row)
        row['read_end_monotonic'] = now()
        if row['read_end_monotonic']-row['read_start_monotonic'] > 10:
            raise RuntimeError('Snapshot took over 10s; no matched pair accepted')
        return row
    before = read()
    validate_pair(before, before)
    cutoff = now()
    while now()-cutoff < 3:
        packet = camera.read_packet()
        if packet is not None:
            stamp, frame = packet
            if cutoff < stamp <= now() and now()-stamp <= .3:
                break
        sleep(.005)
    else:
        raise RuntimeError('No fresh post-read image; old preview is not accepted')
    if burst_seconds:
        if not 0 < burst_seconds <= 2 or score_image is None:
            raise ValueError('Burst requires 0 < duration <= 2s and an image scorer')
        start=now();last_stamp=stamp;best_score=-math.inf;best=None;samples=[]
        def consider(ts, candidate):
            nonlocal best_score,best
            score=float(score_image(candidate))
            if not math.isfinite(score):raise RuntimeError('Invalid image detail score')
            index=len(samples)
            if save_candidate is not None:save_candidate(index,candidate)
            samples.append(dict(index=index,camera_read_monotonic=ts,detail_score=score))
            if score>best_score:best_score=score;best=(ts,candidate,index)
        consider(stamp,frame)
        while now()-start<3:
            packet=camera.read_packet()
            if packet is not None:
                ts,candidate=packet
                if last_stamp<ts<=now() and now()-ts<=.3:
                    last_stamp=ts;consider(ts,candidate)
            if now()-start>=burst_seconds and len(samples)>=5:break
            if len(samples)>=40:break
            sleep(.005)
        report['photo_burst']=dict(samples=samples,selection='maximum central image detail; not geometry validation',
                                   requested_duration_seconds=burst_seconds,elapsed_seconds=now()-start)
        if len(samples)<5:raise RuntimeError('Fewer than 5 fresh burst frames; no photo pair accepted')
        stamp,frame,index=best
        report['photo_burst']['selected_index']=index
        report['photo_burst']['sharpness_verified']=False
    save_image(frame)
    report['image'] = 'paired_photo.png'
    report['camera_read_monotonic'] = stamp
    after = read()
    report['pair_check'] = validate_pair(before, after)
    report['status'] = ('photo_and_readings_saved' if report['pair_check']['stationary_endpoints']
                        else 'pose_changed_pair_not_accepted')
    return report['pair_check']['stationary_endpoints']


def main(capture=False, port='COM7', reference='current'):
    if not capture:
        print('CHECK ONLY: photo bracketed by PING/READ snapshots. No device opened.'); return 0
    import cv2 as cv
    from camera_stream import CameraStream
    from leap_anchor_multiview import poll_key
    from hand_readonly_check import inspect_bus, require_serial_port
    import dynamixel_sdk as sdk
    folder = ROOT / ('pose_pair_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir()
    report = dict(status='started', port=port, servo_write_commands_sent=0,
                  torque_changes_requested=0, policy_actions_sent=0,
                  timestamp=dt.datetime.now().astimezone().isoformat(),
                  timestamp_limit='Camera driver-read timestamp; no exposure/encoder hardware synchronization.')
    report['reference_label']=reference
    report['reference_geometry_verified']=False
    protocol=direction_protocol(reference)
    direction_mode=protocol is not None
    if protocol is not None:report['manual_direction_protocol']=protocol
    if direction_mode:report['direction_pairs']=[]
    capture_number=0
    accept_space_after=0.
    camera = None
    try:
        require_serial_port(port, report)
        print('READ ONLY. USB + motor power connected; torque must already be OFF.', flush=True)
        print('Close all other motor programs. No cube.', flush=True)
        if reference=='middle_upright':
            print('BEFORE SPACE: with motor power OFF, gently prepare the pictured upright middle reference. Do not force.',flush=True)
            print('Then connect motor power with torque OFF; keep the prepared hand still before SPACE.',flush=True)
            print('Leave the prepared pose still during capture. This recording does NOT install calibration.',flush=True)
        elif not direction_mode:
            print('Do NOT reposition the fingers.',flush=True)
        if direction_mode:
            print('TWO PHOTOS in one window. First SPACE: save current pose. Do not move fingers yet.',flush=True)
            print('Wait for BASELINE SAVED before any manual change. Camera stays supported and still.',flush=True)
        if reference=='middle_upright_side':
            print('SIDE VIEW of the existing upright middle finger: move CAMERA only.',flush=True)
            print('Show the palm side edge and entire middle finger from root to tip. Avoid looking down its length.',flush=True)
            print('No marker requirement. Do not repeat the manual pose setup.',flush=True)
            print('Rest the camera on a stable support before SPACE. A 1s burst follows the first angle read.',flush=True)
            print('Keep camera still until SAVED; multiple frames plus the most detailed frame are saved.',flush=True)
        if reference=='middle_ring_existing_side':
            print('SIDE VIEW: move CAMERA only. Keep the current fingers and base unchanged.',flush=True)
            print('Show MIDDLE and RING motor housings, from their roots to their tips, and a palm edge.',flush=True)
            print('Do not straighten or separate the fingers. This is a photo of the existing OFF pose.',flush=True)
            print('USB + 5V supply remain connected for READS; torque must already be OFF. No motion is sent.',flush=True)
        else:
            print('Aim webcam at whole hand: middle root, tip and thumb visible. No marker gate.', flush=True)
        print(('FIRST SPACE saves baseline; follow the prompt before SECOND SPACE. Keep still during each read. Q exits.'
               if direction_mode else 'SPACE once saves fresh photo + before/after encoders. Keep hand and camera still. Q exits.'), flush=True)
        camera = CameraStream(report)
        title = 'POSE + ENCODERS | READ ONLY'
        cv.namedWindow(title,cv.WINDOW_NORMAL); cv.resizeWindow(title,1100,620)
        last_fresh = time.monotonic(); current = None
        while True:
            packet = camera.read_packet()
            if packet is not None and 0 <= time.monotonic()-packet[0] <= .3:
                current=packet[0]; last_fresh=time.monotonic()
                display=packet[1].copy()
                framing=('SIDE VIEW: palm edge + ENTIRE middle root to tip. Move camera only.'
                         if reference=='middle_upright_side' else 'Whole hand + middle root/tip + thumb. No markers required.')
                if reference=='middle_ring_existing_side':
                    framing='SIDE VIEW: middle + ring housings, roots to tips. Move CAMERA only; fingers unchanged.'
                if direction_mode:
                    framing=('FIRST SPACE: current pose. Keep fingers still.' if capture_number==0 else
                             protocol['overlay'])
                for i,line in enumerate(['READ ONLY: no motor commands or torque changes',
                                         framing,
                                         'SPACE: hold still until this capture completes. Q exits.']):
                    cv.putText(display,line,(10,30+30*i),cv.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),2)
                cv.imshow(title,display)
            if time.monotonic()-last_fresh>3: raise RuntimeError('No fresh preview frame')
            key=poll_key()
            if key in (ord('q'),ord('Q'),27,3): raise KeyboardInterrupt('User stop')
            if key==32:
                if time.monotonic()<accept_space_after:continue
                if current is None or time.monotonic()-current>.3:
                    print('Wait for fresh preview',flush=True); continue
                print('Reading before photo; keep hand and camera still.',flush=True)
                capture_folder=folder/('baseline' if capture_number==0 else 'bent') if direction_mode else folder
                capture_folder.mkdir(exist_ok=True)
                target={} if direction_mode else report
                if direction_mode:
                    target.update(phase='baseline' if capture_number==0 else 'bent',relative_folder=capture_folder.name)
                    report['direction_pairs'].append(target)
                def save_image(frame):
                    ok,png=cv.imencode('.png',frame)
                    if not ok: raise RuntimeError('PNG encode failed')
                    (capture_folder/'paired_photo.png').write_bytes(png.tobytes())
                def inspect(row):
                    require_serial_port(port, row)
                    inspect_bus(sdk,port,row)
                def score_image(frame):
                    gray=cv.cvtColor(frame,cv.COLOR_BGR2GRAY);h,w=gray.shape
                    roi=gray[h//6:5*h//6,w//4:3*w//4]
                    return float(cv.Laplacian(roi,cv.CV_64F).var())
                def save_candidate(index,frame):
                    ok,jpg=cv.imencode('.jpg',frame,[cv.IMWRITE_JPEG_QUALITY,95])
                    if not ok:raise RuntimeError('Burst JPEG encode failed')
                    (capture_folder/f'burst_{index:02d}.jpg').write_bytes(jpg.tobytes())
                accepted=capture_pair(inspect,camera,target,save_image,
                             burst_seconds=1 if reference in ('middle_upright_side','middle_ring_existing_side') or direction_mode else 0,
                             score_image=score_image,save_candidate=save_candidate)
                print('Pair result:',target['status'],target['pair_check'],flush=True)
                if direction_mode:
                    (capture_folder/'result.json').write_text(json.dumps(target,indent=2),encoding='utf-8')
                    if not accepted:raise RuntimeError('Pose changed during capture; direction comparison not accepted')
                    if capture_number==0:
                        report['status']='baseline_saved_waiting_for_bent_pose'
                        (folder/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
                        print('BASELINE SAVED. Turn MOTOR POWER OFF before touching fingers.',flush=True)
                        for instruction in protocol['instructions']:print(instruction,flush=True)
                        print('Reconnect motor power; torque stays OFF. Remove your hand, keep CAMERA still, SECOND SPACE.',flush=True)
                        print('If it cannot move gently, Q exits with the baseline preserved.',flush=True)
                        capture_number=1;current=None;last_fresh=time.monotonic()
                        accept_space_after=time.monotonic()+2
                        continue
                    report['direction_check']=summarize_tip_direction(report['direction_pairs'],protocol['action'])
                    report['status']='two_pose_pairs_saved'
                    print('Direction observation:',report['direction_check'],flush=True)
                break
            time.sleep(.005)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted')
        print('STOP:',report['error'],flush=True)
    finally:
        try:
            if camera is not None: camera.release()
            cv.destroyAllWindows()
        finally:
            payload=json.dumps(report,ensure_ascii=False,indent=2)+'\n'
            (folder/'result.json').write_text(payload,encoding='utf-8')
            (ROOT/'leap_pose_pair_readonly_result.json').write_text(payload,encoding='utf-8')
            if direction_mode:(ROOT/protocol['pointer']).write_text(payload,encoding='utf-8')
            print('Saved:',folder,flush=True)
            print('No motor goals or torque changes sent.',flush=True)
    return 0 if report['status'] in ('photo_and_readings_saved','two_pose_pairs_saved') else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',action='store_true'); parser.add_argument('--port',default='COM7')
    parser.add_argument('--reference',choices=('current','middle_upright','middle_upright_side','middle_ring_existing_side','middle_tip_direction','middle_pip_order'),default='current')
    args=parser.parse_args()
    raise SystemExit(main(args.capture,args.port,args.reference))
