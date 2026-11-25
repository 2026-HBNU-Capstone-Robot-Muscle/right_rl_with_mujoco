"""Explicit empty-hand check of the revised simulated start; never runs SAC.

Reuses the established sequence runner's preload, health checks, shutdown and
gain restoration. No motor port is opened without --execute and local SPACE.
"""
import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import sequence_flip_checked as checked
ROOT=Path(__file__).resolve().parent
RESET_SHA='681cfd3bf4eacd67b537a85600b3f9b60ca388776520332f2e163bd7ddf71e21'
SIM_TO_ID=np.array([5,6,7,8,9,10,11,12,13,14,15,16,1,2,3,4])

def target_pose():
    path=ROOT/'sac_supported_reset_result.json';content=path.read_bytes()
    if hashlib.sha256(content).hexdigest()!=RESET_SHA:raise ValueError('Reviewed simulated start changed')
    document=json.loads(content)
    if not document['all_passed']:raise ValueError('Reset validation failed')
    angles=np.asarray(document['candidates'][0]['ctrl'],dtype=float)
    if angles.shape!=(16,) or not np.isfinite(angles).all():raise ValueError('Invalid start angles')
    raw=np.empty(16,dtype=int);raw[SIM_TO_ID-1]=np.rint((angles+np.pi)/checked.TICK).astype(int)
    return checked.validate_raw(raw)

def plan(initial,target):
    initial=checked.validate_raw(initial);target=checked.validate_raw(target)
    if np.max(np.abs(target-initial))*checked.TICK>2.5:raise ValueError('New start exceeds 2.5rad from current position')
    relief=initial.copy();indices=np.array([4,8,12]);floor=int(np.ceil((np.pi+.65)/checked.TICK))
    relief[indices]=np.maximum(relief[indices],floor)
    return [('raise roots away from support',relief),('revised simulated start',target),('hold same start, EMPTY',target.copy())]

def wait_space():
    import msvcrt
    print('EMPTY HAND: secure base; clear fingers, cables and cube. SPACE here starts checks; Q cancels.',flush=True)
    while True:
        if msvcrt.kbhit():
            key=msvcrt.getwch().lower()
            if key==' ':return
            if key in ('q','\x1b'):raise KeyboardInterrupt('User stop')
        time.sleep(.02)

def main(*, capture=False):
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    if args.execute:
        from hand_motor_mapping import reject_legacy_sac_execution
        reject_legacy_sac_execution()
    target=target_pose()
    print('REVISED SAC START CAPTURE: empty preparation -> PLACE/photo -> REMOVE -> OFF. No policy.'
          if capture else 'REVISED SAC START CHECK: empty hand only; no cube, no flip, no SAC actions.',flush=True)
    print('450mA and original sequence PID; root clearance, new pose, PLACE/photo, REMOVE, OFF.'
          if capture else '450mA and original sequence PID; root clearance first, new pose, about 2s hold, OFF.',flush=True)
    print('Goal ticks by motor ID 1..16:',target.tolist(),flush=True)
    if not args.execute:
        print('CHECK ONLY: target validated; no motor port opened.');return 0
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='not_started',phase='preflight',
                mode='revised_start_capture' if capture else 'revised_start_empty_only',reset_manifest_sha256=RESET_SHA,policy_used=False,empty_check=not capture,
                policy_actions_sent=0,all_torque_off_verified=False,settings_restored=False)
    bus=None;opened=False;recorder=None
    try:
        if capture:
            from sequence_vision_recorder import SequenceVisionRecorder
            recorder=SequenceVisionRecorder(ROOT,report,anchor_mode=True)
        else:
            wait_space()
        import dynamixel_sdk as sdk
        bus=checked.Bus(sdk,report)
        if not bus.port.openPort():raise RuntimeError('Cannot open COM7')
        opened=True
        if not bus.port.setBaudRate(3000000):raise RuntimeError('Cannot set 3000000 baud')
        initial=bus.preflight();poses=plan(initial['position'],target)
        for n in (3,2,1):
            checked.check_cancel();print(f'Starting in {n}s; Q/Esc/Ctrl+C stops.',flush=True);time.sleep(1)
        fresh=bus.preflight()
        if np.max(np.abs(fresh['position']-initial['position']))*checked.TICK>.025:raise RuntimeError('Hand moved during countdown')
        callback=None
        if capture:
            from leap_sac_start_capture import capture_placement
            callback=lambda bus,command,report: capture_placement(bus,command,report,recorder)
        checked.run(bus,fresh,poses,report,empty_check=True,prepared_callback=callback)
        print('New start placement saved; cube removed; torque OFF verified.' if capture else
              'Revised start check complete. Report whether a motor housing touched the glass/wood.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted',failed_phase=report.get('phase'))
        print('STOP:',report['error'],flush=True)
    finally:
        if bus is not None and opened:
            try:
                torque=bus.block(64,1,[('torque',64,1)])['torque'];report['final_torque_states']=torque.tolist()
                report['all_torque_off_verified']=bool(np.all(torque==0))
            except Exception as exc:report['final_check_error']=str(exc);report['all_torque_off_verified']=False
            if not report['all_torque_off_verified']:print('CUT MOTOR POWER NOW: OFF unverified.',flush=True)
            bus.port.closePort()
        if recorder is not None:
            try:
                report['vision_recording']=recorder.finish()
                print('Vision saved:',report['vision_recording']['folder'],flush=True)
            except Exception as exc:
                report['vision_save_error']=str(exc)
        stamp=dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        stem='leap_sac_start_capture' if capture else 'leap_sac_start_empty'
        for path in (ROOT/f'{stem}_result.json',ROOT/f'{stem}_{stamp}.json'):
            path.write_text(json.dumps(report,ensure_ascii=False,indent=2,default=float),encoding='utf-8')
        print('Saved:',ROOT/f'{stem}_result.json',flush=True)
    expected='revised_start_capture_complete' if capture else 'empty_check_complete'
    return 0 if report.get('status')==expected and report['all_torque_off_verified'] and report['settings_restored'] and not report.get('vision_save_error') else 1

if __name__=='__main__':raise SystemExit(main())
