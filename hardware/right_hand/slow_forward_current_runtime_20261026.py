"""User-triggered EMPTY thumb remainder with screening from actual OFF angles."""
import argparse
import datetime as dt
import json
from pathlib import Path
import time
import numpy as np
import hand_motor_mapping as mapping
import sequence_flip_checked as checked
from leap_roots_empty import check_off
from leap_thumb_prefix_empty import Bus as ScheduledBus,record_off_endpoint
from quarter_motion_runner_20261025 import run
from thumb_live_start_plan import CurrentPlan,PLAN_VERSION,validate_current_family,validate_off_state
from thumb_live_start_gate import stable_off,write_request,run_screen,validate_proof

ROOT=Path(__file__).resolve().parent


class Bus(ScheduledBus):
    planner=staticmethod(validate_current_family)
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof)
        self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True

    def write(self,addr,size,values,verify=True):
        emergency_off=addr==64 and size==1 and np.array_equal(values,np.zeros(16,dtype=int))
        if not getattr(self,'screen_approved',False) and not emergency_off:
            raise RuntimeError('No matched current-pose proof; no write')
        return super().write(addr,size,values,verify=verify)


def prepare_and_run(bus,folder,report,*,cancel,sleep=time.sleep,now=time.monotonic,
                    screener=run_screen,plan_type=CurrentPlan,prepared_callback=None,phase_callback=None,
                    release_callback=None):
    # Acquisition and simulation are all torque OFF and before configuration.
    validator=getattr(plan_type,'validate_off_state',validate_off_state)
    state=stable_off(bus,report,cancel=cancel,sleep=sleep,now=now,validator=validator)
    request=write_request(folder,state,plan_type.PLAN_VERSION,validator);plan=plan_type(request)
    proof=Path(folder)/'current_start_screen.json'
    report.update(phase='screen_current_pose',request_file=str(request),plan_version=plan_type.PLAN_VERSION)
    print('Checking the current OFF pose and preparation path in simulation. Keep fingers still.',flush=True)
    screener(request,proof,cancel=cancel,sleep=sleep,now=now)
    bus.approve(plan,proof)
    report['current_start_screen']=str(proof)
    report['current_start_screen_accepted']=True
    fresh=bus.preflight();plan.validate_start(fresh['position']);check_off(fresh,fresh['position'])
    report['phase']='countdown'
    for n in range(3,0,-1):
        print(f'Starting in {n}s; EMPTY hand. Q stops.',flush=True)
        for _ in range(20):cancel();sleep(.05)
    initial=bus.preflight();plan.validate_start(initial['position']);check_off(initial,initial['position'])
    if np.any(abs(initial['position']-fresh['position'])>4):raise RuntimeError('Hand moved during countdown; no writes')
    run(bus,initial,report,cancel=cancel,sleep=sleep,now=now,
        prepared_callback=prepared_callback,phase_callback=phase_callback,release_callback=release_callback)


def main(plan_type=CurrentPlan,*,bus_type=Bus,screener=run_screen):
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    print(getattr(plan_type,'TITLE','EMPTY THUMB PREPARATION v2: fresh OFF -> path screen -> thumb -> hold -> OFF.'),flush=True)
    capture_mode=getattr(plan_type,'CAPTURE_PLACEMENT',False)
    empty_release_only=getattr(plan_type,'EMPTY_RELEASE_ONLY',False)
    print('EMPTY ONLY: cube stays OUT for this entire trial.' if empty_release_only else
          'Keep cube OUT until PLACE CUBE.' if capture_mode else 'Keep cube OUT. One terminal SPACE; current pose is measured automatically.',flush=True)
    model_note=getattr(plan_type,'MODEL_LIMITS_DESCRIPTION','native model limits preserved')
    print(f'450mA/original PID; goal step <=2 ticks; {plan_type.ON_TIMEOUT}s motion limit; {model_note}.',flush=True)
    print(f'{plan_type.SAMPLE_PERIOD*1000:g}ms cycles include camera/read/write work; every goal still requires fresh health reads.',flush=True)
    if getattr(plan_type,'CONFIRM_PREVIOUS_GOAL_READBACK',False):
        print('Previous goal packet: hold and READ only; exact command twice within 150ms, otherwise OFF. No goal resend.',flush=True)
    if not args.execute:
        print('CHECK ONLY: no camera, motor port, or simulator process opened.');return 0
    stamp=dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    prefix=getattr(plan_type,'FOLDER_PREFIX','thumb_prepare_current_empty')
    folder=ROOT/f'{prefix}_{stamp}';folder.mkdir()
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='not_started',phase='preview',
        joint_mapping=mapping.metadata(),policy_used=False,physical_flip_verified=False,
        all_torque_off_verified=False,settings_restored=False,hardware_authorized=False,recording_requested=False)
    bus=None;camera=None;opened=False
    try:
        from empty_trial_camera import EmptyTrialCamera
        from camera_frame_signal import CameraImageUnavailable
        camera_type=getattr(plan_type,'CAMERA_TYPE',EmptyTrialCamera)
        camera=camera_type(folder/'camera',report)
        import msvcrt
        print('Leave hand as it is. Show thumb and middle finger; clear fingers/cables.',flush=True)
        print('SPACE in TERMINAL starts OFF checks and simulation, then a 3s motion countdown. Q cancels.',flush=True)
        while True:
            camera.check()
            if msvcrt.kbhit():
                key=msvcrt.getwch()
                if key in ('q','Q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
                if key==' ':
                    try:camera.require_fresh()
                    except CameraImageUnavailable as exc:
                        print(str(exc),flush=True);continue
                    break
            time.sleep(.01)
        report.update(hardware_authorized=True,recording_requested=True,phase='preflight')
        keys=None
        if capture_mode:
            from prepared_placement_capture import TerminalKeys
            keys=TerminalKeys()
        def cancel():
            camera.check()
            if keys is None:checked.check_cancel()
            else:keys.poll()
        import dynamixel_sdk as sdk
        bus=bus_type(sdk,report)
        bus.planner=getattr(plan_type,'validate_admission',validate_current_family)
        if capture_mode:bus.cancel_callback=cancel
        if not bus.port.openPort():raise RuntimeError('Cannot open COM7')
        opened=True
        if not bus.port.setBaudRate(3000000):raise RuntimeError('Cannot set baud')
        bus.preflight()
        callback=phase_callback=release_callback=None
        if empty_release_only:
            protocol=plan_type.CAPTURE_PROTOCOL_TYPE(camera,keys,cancel=cancel)
            callback=protocol.finish;release_callback=protocol.release
        elif getattr(plan_type,'CAPTURE_BEFORE_PHASE',None):
            from grasp_place_capture import GraspCapture
            protocol_type=getattr(plan_type,'CAPTURE_PROTOCOL_TYPE',GraspCapture)
            protocol=protocol_type(camera,keys,cancel=cancel)
            callback=protocol.finish;phase_callback=protocol.place
            if getattr(plan_type,'CONTACT_RELEASE_BEFORE_PHASE',None):release_callback=protocol.release
        elif capture_mode:
            from prepared_placement_capture import capture
            callback=lambda b,c,r:capture(b,c,r,camera,keys,cancel=cancel)
        prepare_and_run(bus,folder,report,cancel=cancel,plan_type=plan_type,screener=screener,
                        prepared_callback=callback,phase_callback=phase_callback,release_callback=release_callback)
        print(getattr(plan_type,'COMPLETE_MESSAGE','Remaining thumb preparation complete. Torque OFF verified; settings restored.'),flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted',failed_phase=report.get('phase'))
        print('STOP:',report['error'],flush=True)
    finally:
        if bus is not None and opened:
            if report.get('settings_restored') and report.get('all_torque_off_verified'):
                record_off_endpoint(bus,report)
            try:report['all_torque_off_verified']=bool(np.all(bus.block(64,1,[('torque',64,1)])['torque']==0))
            except Exception as exc:report.update(all_torque_off_verified=False,final_check_error=str(exc))
            if not report['all_torque_off_verified']:print('CUT MOTOR POWER: torque OFF unverified.',flush=True)
            bus.port.closePort()
        if camera is not None:
            try:report['camera_recording']=camera.finish()
            except Exception as exc:report['camera_save_error']=str(exc)
        payload=json.dumps(report,ensure_ascii=False,indent=2)
        (folder/'result.json').write_text(payload,encoding='utf-8')
        (ROOT/f'leap_{prefix}_result.json').write_text(payload,encoding='utf-8')
        print('Saved:',folder,flush=True)
    return 0 if (report['status']==plan_type.STATUS and report['all_torque_off_verified']
                 and report['settings_restored'] and report.get('camera_recording',{}).get('status')=='complete') else 1


if __name__=='__main__':raise SystemExit('Use leap_slow_forward_placement_20261026.py; no devices opened.')
