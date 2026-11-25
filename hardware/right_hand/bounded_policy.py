"""Rate-limited learned-policy control with explicit bounded/deployment modes.

Bounded commands stay within 32 or 65 ticks; deployment uses original learned joint limits. The outer runner
owns health checks, the total ON deadline, torque OFF and RAM restoration.
"""
import time
import numpy as np
import encoder_policy_shadow as shadow
import hand12_posture_target_100 as motion
from vision_diagnostics import frame_metrics,display_line

TICK=shadow.TICK
RADIUS_TICKS=int(np.floor(.05/TICK))
STEP_TICKS=8
CONTROL_PERIOD_S=float(shadow.limits.CTRL_DT)


class NoPolicyMovement(RuntimeError):
    """A completed observation window with no policy goal writes."""


class Control:
    def __init__(self, anchor, *, read, send, report, now=time.monotonic, step_ticks=STEP_TICKS, radius_ticks=RADIUS_TICKS, joint_limits=False):
        if step_ticks not in (8,RADIUS_TICKS): raise ValueError('Invalid bounded policy step')
        if radius_ticks not in (32,65): raise ValueError('Invalid bounded policy radius')
        self.radius_ticks=radius_ticks
        self.joint_limits=joint_limits
        self.range_label='learned joint limits' if joint_limits else f'+/-{radius_ticks*TICK:.2f}rad'
        self.measured_radius_rad=.075 if radius_ticks==32 else .125
        self.step_ticks=step_ticks
        self.anchor=np.array(anchor,dtype=int,copy=True)
        self.low=(np.ceil(motion.base.sim_to_hw(shadow.limits.SAFE_LOWERS)/TICK).astype(int)
                  if joint_limits else self.anchor-radius_ticks)
        self.high=(np.floor(motion.base.sim_to_hw(shadow.limits.SAFE_UPPERS)/TICK).astype(int)
                   if joint_limits else self.anchor+radius_ticks)
        if joint_limits and (np.any(self.anchor<self.low) or np.any(self.anchor>self.high)):
            raise ValueError('Initial policy target outside learned joint limits')
        self.command=self.anchor.copy()
        self.read_callback,self.send,self.report,self.now=read,send,report,now
        self.last_write=now()
        self.deadline=float('inf')
        self.pace=motion.TrackingPace(report,'bounded_policy_motion',settled_error=.035)
        report.update(policy_actions_sent=0,policy_write_attempts=0,
            policy_joint_limits=joint_limits,
            policy_command_radius_rad=None if joint_limits else self.radius_ticks*TICK,
            policy_command_radius_ticks=None if joint_limits else self.radius_ticks,
            policy_measured_radius_rad=None if joint_limits else self.measured_radius_rad,policy_step_ticks=self.step_ticks,
            policy_command_low_hw_raw=self.low.tolist(),policy_command_high_hw_raw=self.high.tolist(),
            policy_control_period_s=CONTROL_PERIOD_S,policy_action_scale=float(shadow.limits.ACTION_SCALE),
            policy_trial_samples=[])

    def read(self):
        state=self.read_callback(self.command,self.low,self.high)
        q=shadow.hw_to_sim(state['position']*TICK)
        if (np.any(q<shadow.limits.SAFE_LOWERS) or np.any(q>shadow.limits.SAFE_UPPERS)
                or (not self.joint_limits and np.max(np.abs(state['position']-self.anchor))*TICK>self.measured_radius_rad)):
            raise RuntimeError('Measured joint outside bounded policy trial envelope')
        return state

    def apply(self, action, frame_stamp):
        action=np.asarray(action,float)
        if action.shape!=(16,) or not np.isfinite(action).all() or np.max(np.abs(action))>1.000001:
            raise ValueError('Invalid policy action; no write')
        state=self.read()  # Recheck AFTER vision/inference, immediately before writing.
        stamp=self.now()
        if stamp>=self.deadline or stamp-frame_stamp>.2: return None
        error=np.abs(state['position']-self.command)*TICK
        if not self.pace.ready(float(np.max(error)),stamp,int(np.argmax(error))+1): return None
        if stamp-self.last_write<CONTROL_PERIOD_S: return None
        old_q=shadow.hw_to_sim(self.command*TICK)
        proposed=np.clip(old_q+shadow.limits.ACTION_SCALE*action,
                         shadow.limits.SAFE_LOWERS,shadow.limits.SAFE_UPPERS)
        target=np.rint(motion.base.sim_to_hw(proposed)/TICK).astype(int)
        range_limited=np.flatnonzero((target<self.low)|(target>self.high))+1
        target=np.clip(target,self.low,self.high)
        step_limited=np.flatnonzero(np.abs(target-self.command)>self.step_ticks)+1
        target=self.command+np.clip(target-self.command,-self.step_ticks,self.step_ticks)
        target_q=shadow.hw_to_sim(target*TICK)
        if np.any(target_q<shadow.limits.SAFE_LOWERS) or np.any(target_q>shadow.limits.SAFE_UPPERS):
            raise RuntimeError('Rounded policy command outside joint limits')
        if np.array_equal(target,self.command):
            self.report['policy_unchanged_target_count']=self.report.get('policy_unchanged_target_count',0)+1
            return np.zeros(16)
        self.report['policy_write_attempts']+=1
        self.send(target)  # Exact register readback; failure propagates to OFF cleanup.
        self.command=target
        self.last_write=self.now()
        self.report['policy_actions_sent']+=1
        # Feed back the applied, rate/bound-limited action, not the unsent proposal.
        effective=(target_q-old_q)/shadow.limits.ACTION_SCALE
        self.report['policy_trial_samples'].append(dict(
            action=action.tolist(),applied_action=effective.tolist(),
            range_limited_motor_ids=range_limited.tolist(),step_limited_motor_ids=step_limited.tolist(),
            command_hw_raw=target.tolist(),frame_age_ms=(stamp-frame_stamp)*1000))
        return effective


def run(control,camera,policy,report,*,now=time.monotonic,sleep=time.sleep,cancel=motion.base.check_cancel,
        duration_s=10,cube_loaded=False):
    if duration_s not in (5,10,20) or (duration_s==20 and not control.joint_limits):
        raise ValueError('Unsupported policy trial duration')
    began=last_frame=last_print=now()
    control.deadline=began+duration_s
    report['policy_duration_s']=duration_s
    report['policy_cube_loaded']=cube_loaded
    builder=shadow.ob.ObsBuilder()
    last_action=np.zeros(16)
    count=0
    skips=report['policy_skip_counts']={}
    report['policy_inference_count']=0
    report['policy_camera_frames']=0
    report['policy_eligible_frames']=0
    report['policy_vision_diagnostics']=[]
    first_cube_position=None
    while now()-began<duration_s:
        cycle_started=now()
        cancel()
        input_status='waiting_for_frame'
        state=control.read()
        if cube_loaded:
            error=np.abs(state['position']-control.command)*TICK
            control.pace.ready(float(np.max(error)),now(),int(np.argmax(error))+1)
        packet=camera.next(preview_only=True)  # Discard stale frames; never drive from them.
        if packet is not None:
            stamp,frame=packet
            last_frame=now()
            pose=camera.detect(frame)
            metrics=frame_metrics(pose,(now()-stamp)*1000,report.get('last_marker_detection',{}))
            reason=metrics['reason']
            if reason is None:
                cube_position=-np.asarray(pose['palm_minus_cube'],float)
                if first_cube_position is None: first_cube_position=cube_position.copy()
                rotation=shadow.ob.quat_to_mat(np.asarray(pose['cube_quat_world'],float))
                metrics['estimated_target_face_error_deg']=float(np.degrees(np.arccos(np.clip(rotation[2,2],-1.,1.))))
                metrics['estimated_center_displacement_mm']=float(np.linalg.norm(cube_position-first_cube_position)*1000)
            report['policy_camera_frames']+=1
            if reason is None: report['policy_eligible_frames']+=1
            report['policy_vision_diagnostics'].append(dict(
                time_s=now()-began,**metrics,
                palm_minus_cube_m=np.asarray(pose['palm_minus_cube']).tolist() if reason is None else None,
                cube_quat_world=np.asarray(pose['cube_quat_world']).tolist() if reason is None else None,
                detection=dict(report.get('last_marker_detection',{}))))
            if reason:
                count=0
                builder.reset()
                builder.set_last_act(last_action)
                skips[reason]=skips.get(reason,0)+1
            else:
                obs=builder.build(shadow.hw_to_sim(state['position']*TICK),
                    shadow.hw_to_sim(control.command*TICK),pose['palm_minus_cube'],
                    np.zeros(3),pose['cube_quat_world'],shadow.ob.FACE_GOAL_QUATS[0])
                count+=1
                if count>=shadow.ob.HISTORY_LEN:
                    if obs.shape!=(108,) or not np.isfinite(obs).all():
                        raise ValueError('Invalid policy observation')
                    action=policy(obs)
                    report['policy_inference_count']+=1
                    applied=control.apply(action,stamp)
                    if applied is not None:
                        last_action=applied
                        builder.set_last_act(last_action)
                else: reason='history_warmup'
            input_status=reason or 'ready'
            remember=getattr(camera,'record_policy_frame',None)
            if remember is not None: remember(frame,pose,metrics)
            camera.show(frame,[('POLICY | CUBE ON HAND | HANDS CLEAR | Q stops' if cube_loaded else
                               'BOUNDED POLICY MOTION | EMPTY HAND | Q stops'),
                f'{max(0,duration_s-(now()-began)):.1f}s | writes {report["policy_actions_sent"]}',
                f'Input: {reason or "ready"} | range {control.range_label}',
                display_line(metrics)])
        elif now()-last_frame>.2:
            count=0
            builder.reset()
            builder.set_last_act(last_action)
        if now()-last_print>=1:
            print(f'policy_motion: sent={report["policy_actions_sent"]}; input={input_status}; '
                  f'{max(0,duration_s-(now()-began)):.1f}s left',flush=True)
            last_print=now()
        key=camera.key()
        if key in (ord('q'),ord('Q'),27,3): raise KeyboardInterrupt('User stopped policy trial')
        # The cycle already checked motor health above. A second read is only
        # needed in Control.apply, immediately before a new goal write.
        if now()-last_frame>2:
            raise RuntimeError('No fresh camera for 2s; held last goal, now stopping')
        sleep(max(0.,CONTROL_PERIOD_S-(now()-cycle_started)))
    report['bounded_policy_trial_complete']=True
    report['policy_eligible_frame_percent']=100*report['policy_eligible_frames']/max(1,report['policy_camera_frames'])
    valid=[row for row in report['policy_vision_diagnostics'] if row['reason'] is None]
    if valid:
        report['policy_pose_estimate_summary']=dict(
            target_face=0,initial_face_error_deg=valid[0]['estimated_target_face_error_deg'],
            final_face_error_deg=valid[-1]['estimated_target_face_error_deg'],
            min_face_error_deg=min(row['estimated_target_face_error_deg'] for row in valid),
            max_center_displacement_mm=max(row['estimated_center_displacement_mm'] for row in valid),
            physical_success_verified=False,note='Camera estimates; hand/world registration remains a visual candidate')
    print(f'Vision eligible: {report["policy_eligible_frames"]}/{report["policy_camera_frames"]} frames; '
          f'policy inferences: {report["policy_inference_count"]}; writes: {report["policy_actions_sent"]}',flush=True)
    if not report['policy_actions_sent']:
        raise NoPolicyMovement(f'No policy movement in {duration_s}s: '+str(skips))
