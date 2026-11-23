"""Photograph a running grip using its saved health samples, no motor API."""
from pathlib import Path
import argparse, datetime as dt, hashlib, json, time
ROOT = Path(__file__).resolve().parent


def current(pattern='measured_grip_hold_*/live_state.json',phase='measured_grip_hold'):
    files=sorted(ROOT.glob(pattern),key=lambda p:p.stat().st_mtime,reverse=True)
    if not files: raise RuntimeError('파지 실행파일이 유지 상태일 때 확인할 수 있습니다.')
    path=files[0];r=json.loads(path.read_bytes())
    if ((path.parent/'result.json').exists() or not r.get('active')
        or r.get('phase')!=phase
        or any(len(r.get(k,[]))!=16 for k in ('torque','hardware_error','actual_ticks','goal_ticks'))
        or any(v!=1 for v in r['torque']) or any(r['hardware_error'])
        or not 0<=time.monotonic()-r['sample_monotonic']<=1.):
        raise RuntimeError('현재 토크ON 파지 유지 표본이 없습니다. 실행창을 종료하지 말고 유지해 주세요.')
    return path,r


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--capture',action='store_true')
    profile=ap.add_mutually_exclusive_group()
    profile.add_argument('--small-cycle',action='store_true')
    profile.add_argument('--quarter-grip',action='store_true');args=ap.parse_args()
    if not args.capture:
        print('CHECK ONLY: no camera or serial port opened.');return 0
    import cv2 as cv
    from camera_stream import CameraStream
    refpath=ROOT/'right_hand_rotation_20261117/user_measured_grip/reference.json'
    target=json.loads(refpath.read_bytes())['holding_goal_ticks_readonly']
    pattern,phase='measured_grip_hold_*/live_state.json','measured_grip_hold'
    if args.small_cycle:
        from measured_grip_small_cycle_plan_20261117 import SmallCycle
        sequence=SmallCycle();target=sequence.goals[-1].tolist()
        pattern,phase='measured_grip_small_cycle_*/live_state.json','small_cycle_hold'
    if args.quarter_grip:
        pattern,phase='quarter_fixed_grip_trial_*/live_state.json','quarter_fixed_grip_hold'
    def latest():return current(pattern,phase)
    folder=ROOT/('measured_grip_view_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    report=dict(status='started',motor_reads=0,motor_writes=0,torque_changes=0,reference_sha256=hashlib.sha256(refpath.read_bytes()).hexdigest())
    report['goal_profile']='small_cycle_observation_end' if args.small_cycle else 'fixed_measured_grip'
    if args.quarter_grip:report['goal_profile']='quarter_fixed_grip'
    camera=None
    try:
        latest()  # Check ON holding before opening any camera.
        folder.mkdir()
        camera=CameraStream(report,timeout=10.)
        began=time.monotonic();packet=None
        while time.monotonic()-began<3:
            path,before=latest()
            candidate=camera.read_packet()
            if candidate and before['sample_monotonic']<=candidate[0] and 0<=time.monotonic()-candidate[0]<=.3:
                packet=candidate;break
            time.sleep(.02)
        if packet is None:raise RuntimeError('No fresh camera frame')
        stamp,frame=packet
        ok,data=cv.imencode('.png',frame)
        if not ok:raise RuntimeError('PNG encode failed')
        (folder/'held_cube.png').write_bytes(data.tobytes())
        began=time.monotonic()
        while True:
            afterpath,after=latest()
            if afterpath!=path:raise RuntimeError('Grip session changed during photograph')
            if after['sample_monotonic']>stamp:break
            if time.monotonic()-began>1.:raise RuntimeError('No fresh post-photo motor sample')
            time.sleep(.02)
        if before['goal_ticks']!=target or after['goal_ticks']!=target:
            raise RuntimeError('Running target differs from the source-bound observation target')
        drift=max(abs(a-b) for a,b in zip(before['actual_ticks'],after['actual_ticks']))
        report.update(status='photo_and_ON_hold_samples_saved',image='held_cube.png',
            running_session=str(path.parent.relative_to(ROOT)),before=before,after=after,
            camera_driver_read_monotonic=stamp,maximum_endpoint_change_ticks=drift,
            geometry_calibration_verified=False,physical_rotation_verified=False,
            camera_encoder_hardware_synchronization=False)
    except Exception as exc:
        report.update(status='stopped',error=str(exc));print(str(exc),flush=True)
    finally:
        if camera is not None:camera.release()
        if folder.exists():(folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False),flush=True)
    return 0 if report['status']=='photo_and_ON_hold_samples_saved' else 1


if __name__=='__main__':raise SystemExit(main())
