"""Camera-only pose check with saved raw evidence. Never opens a motor port."""
import datetime
import json
import time
from pathlib import Path
import numpy as np
from held_policy_shadow import Camera
from vision_diagnostics import frame_metrics,display_line,summarize,geometry_snapshot

ROOT=Path(__file__).resolve().parent


def pose_snapshot(pose):
    """Keep the measured pose, including one-marker diagnostics, for offline analysis."""
    if pose is None: return None
    fields=('n_cube_markers','reproj','R_cube','t_cube','R_hand','t_hand',
            'palm_minus_cube','cube_quat_world','face_up')
    return {name:np.asarray(pose[name]).tolist() for name in fields if name in pose}


def save_image(cv,path,frame):
    ok,data=cv.imencode('.png',frame)
    if not ok: raise RuntimeError('PNG encoding failed')
    path.write_bytes(data.tobytes())


def target_status(record,required_cube_ids):
    required=set(required_cube_ids or ())|{40}
    missing=sorted(required-set(record['detected_ids']))
    if not missing: return True,'TARGET FOUND: SPACE records 10s'
    if 40 in missing:
        cube_missing=[i for i in missing if i!=40]
        detail=' | cube IDs also missing: '+', '.join(map(str,cube_missing)) if cube_missing else ''
        return False,'SHOW HAND ID40: keep its marker visible, move your hand out of view'+detail
    return False,'ROTATE WHOLE CUBE: missing IDs '+', '.join(map(str,missing))


def sweep_summary(rows,duration):
    """Time-local visibility, so a good static start cannot hide a blind turn."""
    windows=[]
    for start in range(0,duration,2):
        part=[r for r in rows if start<=r['time_s']<start+2]
        windows.append(dict(start_s=start,end_s=start+2,frames=len(part),
                            eligible=sum(r['reason'] is None for r in part),
                            hand_visible=sum(40 in r['detected_ids'] for r in part),
                            two_cube_markers=sum(r['n_cube_markers']>=2 for r in part)))
    return windows


def main(required_cube_ids=None,motion_sweep=False,sweep_seconds=20,external_motion=False):
    required_cube_ids=tuple(sorted(set(required_cube_ids or ())))
    if required_cube_ids and (len(required_cube_ids)!=2 or any(i not in range(6) for i in required_cube_ids)):
        raise ValueError('Specify two distinct cube IDs from 0 to 5')
    if motion_sweep and not 5<=sweep_seconds<=120:
        raise ValueError('Sweep duration must be 5..120 seconds')
    if external_motion and not motion_sweep:
        raise ValueError('External motion observation requires --sweep')
    tag=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder=ROOT/(('vision_sweep_' if motion_sweep else 'vision_check_')+tag)
    report=dict(timestamp=datetime.datetime.now().astimezone().isoformat(),
                motor_port_opened=False,policy_actions_sent=0,status='preview',frames=[],
                required_cube_ids=list(required_cube_ids),wrong_target_space_count=0,
                motion_sweep=motion_sweep,external_motion_observation=external_motion)
    camera=None; examples={}; sweep_examples={}; started=None; wall_clock_offset=None
    duration=sweep_seconds if motion_sweep else 10
    last_row=None; last_stamp=None
    try:
        camera=Camera(report)
        camera.title=f'VISION ONLY | SPACE records {duration}s | NO MOTORS | Q stops'
        camera.cv.namedWindow(camera.title,camera.cv.WINDOW_NORMAL)
        camera.cv.resizeWindow(camera.title,1100,620)
        print(('NO MOTOR PORT in this camera program. Another program controls the hand.' if external_motion
               else 'NO MOTOR PORT. Keep motor power OFF. Show ID40 and two broad cube faces.'),flush=True)
        print(f'SPACE records {duration}s and saves raw frames + numeric diagnostics. Q exits.',flush=True)
        if motion_sweep:
            print(('After SPACE, start the known sequence in a SECOND terminal; keep hands clear.' if external_motion
                   else 'After SPACE, slowly rotate the WHOLE cube along the intended path. Keep fingers clear.'),flush=True)
        if required_cube_ids:
            print(f'TARGET: cube IDs {list(required_cube_ids)} + hand ID40. Rotate the WHOLE cube, not individual layers.',flush=True)
            print('Wait for TARGET FOUND, then SPACE. High reprojection error is OK for this diagnostic capture.',flush=True)
        last_frame=time.monotonic()
        last_prompt=last_frame
        while started is None or time.monotonic()-started<duration:
            packet=camera.next(preview_only=True)
            if packet is not None:
                stamp,frame=packet
                last_frame=time.monotonic()
                pose=camera.detect(frame)
                row=frame_metrics(pose,(time.monotonic()-stamp)*1000,report.get('last_marker_detection',{}))
                last_row,last_stamp=row,stamp
                if started is not None:
                    row['time_s']=time.monotonic()-started
                    row['captured_at_unix_s']=float(stamp+wall_clock_offset)
                    row['pose']=pose_snapshot(pose)
                    report['frames'].append(row)
                    if motion_sweep:
                        bucket=min((duration-1)//10,int(row['time_s']//10))
                        kind='eligible' if row['reason'] is None else 'not_ready'
                        sweep_examples.setdefault((bucket,kind),frame.copy())
                    category=row['reason'] or 'eligible'
                    score=row['reprojection_px'] or 0.
                    old=examples.get(category)
                    if old is None or (category=='reprojection_over_3px_or_invalid' and score>old['score']):
                        examples[category]=dict(frame=frame.copy(),metrics=row.copy(),score=score,
                                                geometry=geometry_snapshot(camera,pose))
                label=row['reason'] or 'READY'
                remaining=f'SPACE: begin {duration}s capture' if started is None else f'{max(0,duration-(time.monotonic()-started)):.1f}s remaining'
                target_line=target_status(row,required_cube_ids)[1] if required_cube_ids else 'VISION ONLY | NO MOTOR COMMANDS'
                if required_cube_ids and started is None and time.monotonic()-last_prompt>=1:
                    print(target_line+' | detected '+str(row['detected_ids']),flush=True)
                    last_prompt=time.monotonic()
                if required_cube_ids:
                    found=target_status(row,required_cube_ids)[0]
                    color=(0,255,0) if found else (0,180,255)
                    hint='FOUND' if found else ('SHOW HAND ID40' if 40 not in row['detected_ids'] else 'ROTATE CUBE')
                    camera.cv.putText(frame,'TARGET '+' + '.join(map(str,required_cube_ids))+': '+hint,
                        (10,frame.shape[0]-55),camera.cv.FONT_HERSHEY_SIMPLEX,1.,color,2)
                camera.show(frame,[target_line,remaining,
                    display_line(row),label,(('External sequence controls motors; hands clear.' if external_motion else
                                              'Slowly rotate WHOLE cube; motor power OFF.') if motion_sweep
                                              else 'Show ID40 + two cube markers; keep cube still during recording.')])
            if time.monotonic()-last_frame>2: raise RuntimeError('No fresh camera for 2s')
            key=camera.key()
            if key in (ord('Q'),3): raise KeyboardInterrupt('User stopped')
            if key==32 and started is None:
                if required_cube_ids and (last_row is None or time.monotonic()-last_stamp>.2 or
                                         not target_status(last_row,required_cube_ids)[0]):
                    report['wrong_target_space_count']+=1
                    hint=target_status(last_row,required_cube_ids)[1] if last_row else 'Waiting for a fresh camera frame'
                    if last_stamp is not None and time.monotonic()-last_stamp>.2: hint='Waiting for a fresh camera frame'
                    print('Not recording yet: '+hint,flush=True)
                    continue
                started=time.monotonic()
                wall_clock_offset=time.time()-started
                report['recording_started_unix_s']=float(started+wall_clock_offset)
                report['status']='recording'
                print(f'Recording {duration}s. Raw examples are saved automatically.',flush=True)
            time.sleep(.005)
        report['status']='complete'
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted')
    finally:
        if camera is not None:
            try: camera.close()
            except Exception as exc: report.update(status='stopped',close_error=str(exc))
        report['summary']=summarize(report['frames'])
        if motion_sweep: report['sweep_windows']=sweep_summary(report['frames'],duration)
        if required_cube_ids:
            report['target_present_frames']=sum(target_status(r,required_cube_ids)[0] for r in report['frames'])
        folder.mkdir(exist_ok=True)
        saved=[]
        for category,example in examples.items():
            path=folder/(category+'.png')
            try:
                save_image(camera.cv,path,example['frame'])
                (folder/(category+'.json')).write_text(json.dumps(dict(metrics=example['metrics'],geometry=example['geometry']),indent=2),encoding='utf-8')
                saved.append(str(path))
            except Exception as exc: report.setdefault('save_errors',[]).append(str(exc))
        for (bucket,kind),frame in sweep_examples.items():
            path=folder/f'{bucket*10:02d}-{min(duration,bucket*10+10):02d}s_{kind}.png'
            try:
                save_image(camera.cv,path,frame)
                saved.append(str(path))
            except Exception as exc: report.setdefault('save_errors',[]).append(str(exc))
        report.update(saved_images=saved,evidence_folder=str(folder))
        data=json.dumps(report,indent=2)
        (folder/'result.json').write_text(data,encoding='utf-8')
        (ROOT/'leap_vision_check_result.json').write_text(data,encoding='utf-8')
        print('Vision summary: '+json.dumps(report['summary']),flush=True)
        print(f'Saved: {folder}',flush=True)
    return 0 if report['status']=='complete' and not report.get('save_errors') else 1


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cube-ids',nargs=2,type=int,choices=range(6),default=None)
    parser.add_argument('--sweep',action='store_true',help='20s hand-driven cube rotation, no motor port')
    parser.add_argument('--sweep-seconds',type=int,default=20,help='recording duration for --sweep, 5..120s')
    parser.add_argument('--external-motion',action='store_true',help='observe another motor program; this process never opens its port')
    args=parser.parse_args()
    raise SystemExit(main(args.cube_ids,motion_sweep=args.sweep,sweep_seconds=args.sweep_seconds,
                          external_motion=args.external_motion))
