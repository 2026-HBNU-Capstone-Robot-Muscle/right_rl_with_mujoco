"""Operator placement -> 5s bounded policy -> operator removal.

No policy goal writes outside the policy stage. Faults propagate to the outer
runner's verified torque-OFF cleanup. This is not an unrestricted manipulation run.
"""
import time
import numpy as np
import bounded_policy
import encoder_policy_shadow as shadow
from vision_diagnostics import frame_metrics,display_line


def run(control,camera,policy,report,*,input_keys,now=time.monotonic,sleep=time.sleep,duration_s=5):
    if duration_s not in (5,20) or (duration_s==20 and not control.joint_limits):
        raise ValueError('20s run requires joint-limit control')
    report.update(loaded_policy_mode=True,cube_placement_confirmed=False,cube_removal_confirmed=False,
                  loaded_trial_complete=False,loaded_stage_events=[])
    control.deadline=now()  # No writes allowed until placement is confirmed.

    def wait_for_operator(phase):
        report['contact_phase']=phase
        input_keys.pending=None  # Never carry an earlier-stage SPACE forward.
        began=last_frame=last_print=now()
        last_valid=None
        consecutive=0
        detail=''
        fixed=control.command.copy()
        writes_before=report.get('policy_write_attempts',0)
        placement=('PLACE CUBE on INDEX/MIDDLE proximal segments only if ID40 + two cube markers stay visible; '
                   if duration_s==20 and control.joint_limits else 'PLACE CUBE NOW; ')
        prompt=(f'{placement}hands clear, then SPACE when vision READY to start {duration_s}s policy motion.'
                if phase=='place' else 'REMOVE CUBE NOW; SPACE confirms removed and turns torque OFF.')
        print(prompt,flush=True)
        report['loaded_stage_events'].append(dict(stage=phase,event='begin',time_s=now()))
        while True:
            input_keys.poll()
            state=control.read()
            if not np.array_equal(state['goal'],fixed): raise RuntimeError('Goal changed during operator wait')
            error=np.abs(state['position']-fixed)*shadow.TICK
            control.pace.ready(float(np.max(error)),now(),int(np.argmax(error))+1)
            packet=camera.next(preview_only=True)
            status='waiting_for_frame'
            if packet is not None:
                stamp,frame=packet
                last_frame=now()
                if phase=='place':
                    pose=camera.detect(frame)
                    metrics=frame_metrics(pose,(now()-stamp)*1000,report.get('last_marker_detection',{}))
                    report['placement_last_vision']=metrics
                    reason=metrics['reason']
                    counts=report.setdefault('placement_vision_counts',{})
                    counts[reason or 'eligible']=counts.get(reason or 'eligible',0)+1
                    detail=display_line(metrics)
                    consecutive=consecutive+1 if reason is None else 0
                    last_valid=stamp if reason is None else None
                    status='READY' if consecutive>=shadow.ob.HISTORY_LEN else reason or 'history_warmup'
                else:
                    status='No marker requirement for removal confirmation'
                camera.show(frame,[prompt,f'{now()-began:.1f}s | {status}',
                    detail,
                    'SPACE in CAMERA or TERMINAL; Q turns torque OFF immediately.'])
            elif now()-last_frame>.2:
                consecutive=0; last_valid=None
            control.read()
            if now()-last_frame>2: raise RuntimeError('No fresh camera for 2s during loaded operator wait')
            key=input_keys.key()
            if key==32:
                ready=last_valid is not None and now()-last_valid<=.2 and consecutive>=shadow.ob.HISTORY_LEN
                if phase=='remove' or ready:
                    report['loaded_stage_events'].append(dict(stage=phase,event='confirmed',time_s=now()))
                    break
                report['placement_space_rejected']=report.get('placement_space_rejected',0)+1
                print('Vision not ready. '+detail+'; keep hands clear, then SPACE when READY.',flush=True)
            if now()-last_print>=1:
                print(f'loaded_policy: {phase}; {status}; {detail}; waiting for SPACE',flush=True)
                last_print=now()
            sleep(.02)
        if report.get('policy_write_attempts',0)!=writes_before:
            raise RuntimeError('Unexpected policy write during operator wait')

    wait_for_operator('place')
    report.update(cube_placement_confirmed=True,cube_location='operator confirmed on hand; hands clear',
                  contact_phase='policy')
    input_keys.pending=None
    report['loaded_stage_events'].append(dict(stage='policy',event='begin',time_s=now()))
    print(f'POLICY MOTION NOW: {duration_s}s, {control.range_label}. Keep hands clear; Q stops.',flush=True)
    pending_error=None
    try:
        bounded_policy.run(control,camera,policy,report,now=now,sleep=sleep,cancel=input_keys.poll,
                           duration_s=duration_s,cube_loaded=True)
    except bounded_policy.NoPolicyMovement as exc:
        pending_error=exc  # Still allow orderly removal when only inference/movement was absent.
    finally:
        control.deadline=now()
    report['loaded_stage_events'].append(dict(stage='policy',event='end',time_s=now()))
    wait_for_operator('remove')
    report.update(cube_removal_confirmed=True,cube_location='operator confirmed removed',contact_phase='finished')
    if pending_error is not None: raise pending_error
    report['loaded_trial_complete']=True
