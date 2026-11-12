"""User-triggered bounded EMPTY preparation observation. No cube or SAC."""
import argparse
import datetime as dt
import json
from pathlib import Path
import time
from collections import deque
import numpy as np
import sequence_flip_checked as checked
from leap_roots_empty import check_off
import hand_motor_mapping as mapping
import thumb_prefix_plan as p

ROOT=Path(__file__).resolve().parent

def validate_offline_review(plan_module=p):
    p=plan_module
    review_path=getattr(p,'REVIEW_PATH',ROOT/'right_hand_preparation_20261021/thumb_prefix_screen.json')
    record=json.loads(review_path.read_text(encoding='utf-8'))
    if (record['source_sha256']!=p.SOURCE_SHA or record['start_ticks']!=p.source_position().tolist()
            or record['last_goal_ticks']!=p.plan(p.source_position())[-1][1].tolist()
            or len(record['static_screens'])!=67):
        raise ValueError('Offline review does not match this bounded observation')
    if hasattr(p,'PLAN_VERSION') and (record.get('plan_version')!=p.PLAN_VERSION
            or record.get('plan_sha256')!=p.plan_digest(p.source_position())):
        raise ValueError('Offline review path/version changed; no device opened')
    for row in record['static_screens']:
        if (row['max_self_overlap_mm']>1e-6 or row['max_support_increase_mm']>1e-6
                or row['thumb_support_overlap_mm']>1e-6
                or row['max_goal_step_ticks']>getattr(p,'MAX_GOAL_STEP',1)):
            raise ValueError('Offline preparation screen did not pass; no device opened')

class Bus(checked.Bus):
    plan_module=p
    planner=staticmethod(p.validate_start)

    def bind(self,raw):
        p=self.plan_module
        self.original=p.validate_start(raw)
        self.pending=[self.original.copy()]
        self.adopted=False

    def adopt(self,raw):
        p=self.plan_module
        raw=p.validate_start(raw)
        if self.adopted or self.pending or np.any(abs(raw-self.original)>4):
            raise ValueError('Invalid or repeated enable-hold adoption')
        self.pending=[raw.copy()]+[q for _,q in p.plan(raw)]
        self.adopted=True

    def write(self,addr,size,values,verify=True):
        if addr==116:
            a=np.asarray(values)
            if (size!=4 or not hasattr(self,'pending') or not self.pending
                    or a.shape!=(16,) or not np.isfinite(a).all()
                    or np.any(a!=np.rint(a)) or not np.array_equal(a,self.pending[0])):
                raise ValueError('Goal differs from the next bounded preparation sample; no write')
        result=super().write(addr,size,values,verify=verify)
        if addr==116:
            self.previous_goal_command=getattr(self,'last_goal_command',None)
            self.last_goal_command=np.asarray(values).copy()
            self.pending.pop(0)
        return result

def record_off_endpoint(bus,report,*,sleep=time.sleep):
    """Best-effort OFF reference; no writes, no automatic start acceptance.

    Torque OFF/restoration already happened. Recording its subsequent encoder
    pose avoids using a loaded ON endpoint as if it were an OFF start.
    """
    report['phase']='post_off_reference'
    report['final_off_reference_valid']=False
    snapshots=[]
    try:
        sleep(.25)
        for _ in range(2):
            s=bus.snapshot()
            snapshots.append({k:v.tolist() for k,v in s.items()})
            if any(np.any(s[k]) for k in ('torque','error','watchdog')):
                raise RuntimeError('OFF reference torque/error/watchdog check failed')
            if (np.any(s['voltage']<38) or np.any(s['voltage']>65)
                    or np.any(s['temperature']>45)):
                raise RuntimeError('OFF reference voltage/temperature check failed')
            sleep(.15)
        first,last=(np.array(s['position']) for s in snapshots)
        report['final_off_endpoint_drift_ticks']=(last-first).tolist()
        report['final_off_reference_valid']=bool(np.all(abs(last-first)<=2))
        if 'final_on_state' in report:
            report['on_to_off_position_change_ticks']=(last-np.array(report['final_on_state']['position'])).tolist()
    except Exception as exc:
        report['off_reference_capture_error']=str(exc)
    finally:
        report['final_off_snapshots']=snapshots

def run(bus,initial,report,*,cancel=checked.check_cancel,sleep=time.sleep,now=time.monotonic,
        prepared_callback=None,phase_callback=None,release_callback=None):
    p=bus.plan_module
    checkpoint=getattr(p,'CAPTURE_BEFORE_PHASE',None)
    if checkpoint and (phase_callback is None or prepared_callback is None):
        raise RuntimeError('Loaded approach needs both PLACE and REMOVE callbacks; no writes')
    release_phase=getattr(p,'CONTACT_RELEASE_BEFORE_PHASE',None)
    if release_phase and release_callback is None:
        raise RuntimeError('Measured contact release needs an observation callback; no writes')
    original=p.validate_start(initial['position']);check_off(initial,original)
    bus.bind(original)
    report.update(initial={k:v.tolist() for k,v in initial.items()},source_sha256=p.SOURCE_SHA,
        source_path=str(p.SOURCE),start_difference_ticks=(original-p.source_position()).tolist(),
        settings_restored=False,all_torque_off_verified=False,thumb_max_goal_travel_ticks=p.MAX_THUMB_TRAVEL)
    report['cycle_pacing']=('processing_included' if hasattr(p,'SAMPLE_PERIOD') else 'fixed_sleep_after_processing')
    report['requested_sample_period_s']=getattr(p,'SAMPLE_PERIOD',None)
    cancel();check_off(bus.snapshot(),original)
    try:
        report['phase']='configure_off'
        for addr,key in ((80,'d_gain'),(82,'i_gain'),(84,'p_gain')):bus.write(addr,2,checked.GAINS[key])
        bus.write(102,2,checked.CAP)
        bus.write(108,4,np.zeros(16,dtype=int));bus.write(112,4,np.zeros(16,dtype=int))
        bus.write(116,4,original,verify=False)
        previous=None
        for _ in range(2):
            cancel();s=bus.snapshot()
            report.setdefault('off_preload_snapshots',[]).append({k:v.tolist() for k,v in s.items()})
            check_off(s,original,configured=True)
            if previous is not None and any(np.any(abs(s[k]-previous[k])>1) for k in ('position','goal')):
                raise RuntimeError('OFF preload unstable; no enable')
            previous=s;sleep(.02)
        bus.write(98,1,np.full(16,25));cancel();bus.write(64,1,np.ones(16,dtype=int))
        report['enabled_at']=now();report['phase']='enable_hold';previous=None
        for _ in range(2):
            cancel();s=bus.snapshot()
            report.setdefault('enable_snapshots',[]).append({k:v.tolist() for k,v in s.items()})
            checked.health(s,s['goal'])
            if any(np.any(abs(a)>4) for a in (s['position']-original,s['goal']-original,s['goal']-s['position'])):
                raise RuntimeError('Enable changed raw hold beyond four ticks; OFF')
            if previous is not None and any(np.any(abs(s[k]-previous[k])>1) for k in ('position','goal')):
                raise RuntimeError('Enable hold unstable; OFF')
            previous=s;sleep(.02)
        command=s['goal'].copy();bus.adopt(command)
        # Only an opted-in current plan may confirm an old packet. Enable and
        # preload remain strict; each actual accepted command stays unchanged.
        bus.goal_readback_confirmation=bool(getattr(p,'CONFIRM_PREVIOUS_GOAL_READBACK',False))
        if bus.goal_readback_confirmation:
            report['goal_readback_policy']=dict(version='exact_previous_packet_fresh_confirm_v1',
                confirm_seconds=.15,max_fresh_reads=3,exact_reads_required=2,
                maximum_previous_step_ticks=2,goal_write_retries=0)
        fixed=command.copy();rows=p.plan(command)
        report.update(accepted_enable_goal=command.tolist(),enable_goal_shift_ticks=(command-original).tolist(),
            final_command=rows[-1][1].tolist(),thumb_travel_ticks=(rows[-1][1][:4]-command[:4]).tolist())
        bus.write(116,4,command)

        held_guard=None
        if getattr(p,'HELD_JOINT_GUARD',False):
            from held_joint_guard import HeldJointGuard
            held_guard=HeldJointGuard(s['position'],command)
            report['held_joint_guard']=held_guard.VERSION

        progress_history={};paused_seconds=0.;checkpoint_done=False;release_done=False
        def observe(phase):
            cycle_start=now()
            cancel()
            timeout=getattr(p,'ON_TIMEOUT',35)
            if cycle_start-report['enabled_at']-paused_seconds>timeout:
                raise RuntimeError(f'Preparation exceeded {timeout}s; OFF')
            frozen=p.frozen_for_phase(phase) if hasattr(p,'frozen_for_phase') else p.FROZEN
            progress=(p.progress_for_phase(phase) if hasattr(p,'progress_for_phase')
                      else np.arange(4) if phase=='stage_1_thumb_prefix' else np.array([],dtype=int))
            def check_snapshot(state):
                if held_guard is not None:
                    from held_joint_guard import HeldJointFault
                    try:held_guard.check(state['position'],command,frozen,phase)
                    except HeldJointFault as exc:
                        report['fixed_hold_fault']=exc.details
                        raise
                elif np.any(abs(state['position'][frozen]-command[frozen])>20):
                    raise RuntimeError('A fixed finger joint moved over 20 ticks; no target advance')
                if len(progress):
                    recent=progress_history.setdefault(phase,deque(maxlen=20))
                    recent.append(state['position'][progress].copy())
                    if len(recent)==20:
                        stopped=(np.ptp(np.array(recent),axis=0)<=2)&(abs(command[progress]-state['position'][progress])>=64)
                        if np.any(stopped):
                            motor=int(progress[np.flatnonzero(stopped)[0]])+1
                            raise RuntimeError(f'ID{motor} no progress over 20 fresh samples with >=64 tick error; no target advance')
            state,errors=checked.sample(bus,command,report,phase,snapshot_check=check_snapshot)
            return state,errors,cycle_start

        def finish_cycle(cycle_start):
            period=getattr(p,'SAMPLE_PERIOD',None)
            if period is None:sleep(.05)
            else:
                # Schedule each cycle independently. No catch-up bursts and no
                # skipped targets; each next goal still requires a fresh read.
                remaining=max(0.,period-(now()-cycle_start))
                if remaining:sleep(remaining)

        for i,(phase,target) in enumerate(rows,1):
            if phase==release_phase and not release_done:
                report['phase']='release_checkpoint'
                state,_,_=observe('release_checkpoint')
                contact_cases=[s for s in bus.proof['static_screens'] if s['initial_self_contacts_mm']]
                threshold=(min(s['contact_clear_pip_goal_ticks'] for s in contact_cases)-p.CONTACT_RELEASE_MARGIN_TICKS
                           if contact_cases else None)
                report['contact_release_threshold_ticks']=threshold
                report['contact_release_actual_ticks']=int(state['position'][14])
                if threshold is not None and state['position'][14]>threshold:
                    raise RuntimeError(f'ID15 has not reached the certified release: actual={state["position"][14]}, need<={threshold}; no thumb/opening advance')
                began=now();release_callback(bus,command.copy(),report,held_guard)
                elapsed=now()-began
                if elapsed>100 or not report.get('contact_release_operator_confirmed'):
                    raise RuntimeError('Contact release was not confirmed; no thumb/opening advance')
                paused_seconds+=elapsed;report['contact_release_pause_seconds']=elapsed
                report['operator_pause_seconds_total']=paused_seconds;release_done=True
            if phase==checkpoint and not checkpoint_done:
                # The queue still contains the entire loaded segment. Waiting
                # holds the last EMPTY goal; it cannot consume or replace a
                # goal. The protocol requires fresh SPACE and ongoing health.
                if i==1 or held_guard is None:raise RuntimeError('Invalid placement checkpoint; OFF')
                for _ in range(getattr(p,'CHECKPOINT_SETTLE_SAMPLES',1)):
                    state,_,cycle_start=observe('placement_checkpoint');finish_cycle(cycle_start)
                if np.any(abs(state['position']-command)>getattr(p,'CHECKPOINT_MAX_ERROR_TICKS',4096)):
                    raise RuntimeError('Open preparation has not settled within its placement tolerance; no loaded target advance')
                began=now()
                phase_callback(bus,command.copy(),report,held_guard)
                elapsed=now()-began
                if elapsed>getattr(p,'PLACEMENT_PAUSE_LIMIT',100):
                    raise RuntimeError('Placement checkpoint exceeded its time limit; OFF')
                if not report.get('cube_placement_confirmed'):
                    raise RuntimeError('No explicit cube placement confirmation; no loaded target advance')
                paused_seconds+=elapsed;report['placement_pause_seconds']=elapsed
                report['operator_pause_seconds_total']=paused_seconds
                checkpoint_done=True
            report['phase']=phase
            state,errors,cycle_start=observe(phase)
            bus.write(116,4,target,verify=False)
            if held_guard is not None:held_guard.advance(state['position'],command,target)
            command=target.copy()
            if i%30==0:print(f'{phase}: {i}/{len(rows)}; tracking {max(errors):.3f}rad',flush=True)
            finish_cycle(cycle_start)
        if checkpoint and not checkpoint_done:raise RuntimeError('Placement checkpoint missing from path; OFF')
        report['phase']='stage_2_fixed_hold'
        for _ in range(20):
            state,errors,cycle_start=observe(report['phase']);finish_cycle(cycle_start)
        report['final_on_state']={k:v.tolist() for k,v in state.items()}
        if prepared_callback is not None:prepared_callback(bus,command,report)
        report['status']=getattr(p,'STATUS','empty_thumb_prefix_complete')
    finally:
        bus.off();report['all_torque_off_verified']=True
        bus.write(98,1,np.zeros(16,dtype=int))
        for addr,size,key in ((102,2,'current_cap'),(108,4,'acceleration'),(112,4,'velocity'),
            (98,1,'watchdog'),(80,2,'d_gain'),(82,2,'i_gain'),(84,2,'p_gain')):
            bus.write(addr,size,initial[key])
        report['settings_restored']=True

def main(plan_module=p,bus_type=Bus):
    p=plan_module
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    validate_offline_review(p)
    raw=p.source_position();rows=p.plan(raw)
    print(getattr(p,'TITLE','EMPTY THUMB PREPARATION PREFIX: roots first -> small thumb movement -> hold -> OFF.'),flush=True)
    print(getattr(p,'DESCRIPTION','Thumb IDs1..4 max 96 ticks (8.44deg) from fresh start. No cube, full reset or SAC.'),flush=True)
    if not args.execute:
        print('OFFLINE ONLY: no camera or motor port. Source:',raw.tolist())
        print('Final command:',rows[-1][1].tolist());return 0
    stamp=dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    prefix=getattr(p,'FOLDER_PREFIX','thumb_prefix_empty')
    folder=ROOT/f'{prefix}_{stamp}';folder.mkdir()
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='not_started',phase='preview',
        joint_mapping=mapping.metadata(),policy_used=False,physical_flip_verified=False,
        all_torque_off_verified=False,hardware_authorized=False,recording_requested=False)
    report['plan_version']=getattr(p,'PLAN_VERSION','thumb_prefix_empty_v1')
    bus=None;opened=False;camera=None
    try:
        from empty_trial_camera import EmptyTrialCamera
        camera=EmptyTrialCamera(folder/'camera',report)
        import msvcrt
        print('Leave fingers as they are; EMPTY hand. Show the thumb and middle finger in the camera.',flush=True)
        print('450mA, original sequence PID. One command records camera + encoders. No marker gate.',flush=True)
        print('Hands/cables clear. SPACE in TERMINAL starts checks/countdown; Q/Esc/Ctrl+C stops.',flush=True)
        while True:
            camera.check()
            if msvcrt.kbhit():
                key=msvcrt.getwch()
                if key in ('q','Q','\x1b','\x03'):raise KeyboardInterrupt('User stop')
                if key==' ':camera.require_fresh();break
            time.sleep(.01)
        report['hardware_authorized']=True;report['recording_requested']=True;report['phase']='preflight'
        def cancel():
            camera.check();checked.check_cancel()
        import dynamixel_sdk as sdk
        bus=bus_type(sdk,report)
        if not bus.port.openPort():raise RuntimeError('Cannot open COM7')
        opened=True
        if not bus.port.setBaudRate(3000000):raise RuntimeError('Cannot set baud')
        initial=bus.preflight();p.validate_start(initial['position']);check_off(initial,initial['position'])
        for n in range(3,0,-1):
            print(f'Starting in {n}s; EMPTY hand.',flush=True)
            for _ in range(20):cancel();time.sleep(.05)
        fresh=bus.preflight()
        if np.any(abs(fresh['position']-initial['position'])>4):raise RuntimeError('Hand moved during countdown; no writes')
        run(bus,fresh,report,cancel=cancel)
        print(getattr(p,'COMPLETE_MESSAGE','Small thumb preparation complete. Torque OFF verified; settings restored.'),flush=True)
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
    return 0 if (report['status']==getattr(p,'STATUS','empty_thumb_prefix_complete') and report['all_torque_off_verified']
        and report.get('settings_restored') and report.get('camera_recording',{}).get('status')=='complete') else 1

if __name__=='__main__':raise SystemExit(main())
