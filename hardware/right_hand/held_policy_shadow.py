"""Empty-hand start pose + fixed-goal hold with UNSENT policy inference.
Servo goals come ONLY from the previously tested recovery/start-pose runner.
Camera is read in a background thread so a stalled capture cannot block safety reads.
"""
import datetime
import json
import sys
import threading
import time
import numpy as np
import hand_recover_then_start as motion
import encoder_policy_shadow as shadow


class Camera:
    def __init__(self,report):
        import cv2 as cv
        from camera_stream import CameraStream
        self.cv=cv
        self.detector=None
        self.cap=None
        self.stop=threading.Event(); self.lock=threading.Lock()
        self.packet=None; self.error=None; self.thread=None; self.used=-1
        self.title='FIXED HOLD + UNSENT POLICY | Q stops'
        self.report=report
        from policy_vision_evidence import Evidence
        self.policy_evidence=Evidence()
        if not report.get('prepare_only',False): self._setup_tracking()
        self.cap=CameraStream(report)
        report.update(capture=self.cap.meta)
        self.thread=threading.Thread(target=self._capture,daemon=True)
        self.thread.start()

    def _setup_tracking(self):
        from hand_registration_geometry import candidate
        sys.path.insert(0,str(shadow.PROJECT/'vision/rubik_center'))
        import cube_board as cb
        import track_cube as tc
        self.tc=tc
        cv,report=self.cv,self.report
        registration=candidate()
        tc.PALM_IN_MARKER=np.array(registration['grasp_site_in_marker_m_candidate'])
        tc.HAND_TO_WORLD=np.array(registration['hand_to_world_candidate'])
        report['registration']=registration
        report['registration_depth_verified']=False
        with np.load(shadow.ROOT/'camera1_720.npz') as cal:
            if (int(cal['height']),int(cal['width']))!=(720,1280): raise ValueError('Wrong calibration size')
            self.K,self.dist=cal['K'].copy(),cal['dist'].copy()
        rotations=tuple(json.loads((shadow.ROOT/'cube_face_rot_result.json').read_text())['face_rot'])
        points,ids=cb.object_points(cube_mm=54.5,marker_mm=10.6,layout='centers',face_rot=rotations)
        self.dictionary,_,self.detector=tc.build(marker_mm=10.6,grid=1)
        self.board=cv.aruco.Board(points,self.dictionary,ids)
        from marker_detector import MarkerDetector
        self.detector=MarkerDetector(self.dictionary,ids)
        report['marker_detector']='contour corners + raw dense/contrast/dense/strong-threshold fallbacks; max 5 same-frame passes; decoded board IDs only'
        report['face_rot']=rotations

    def _capture(self):
        seq=0
        try:
            while not self.stop.is_set():
                packet=self.cap.read_packet()
                if packet is None:
                    self.stop.wait(.01)
                    continue
                stamp,frame=packet
                if frame.shape[:2]!=(720,1280): raise RuntimeError('Camera resolution changed')
                seq+=1
                with self.lock: self.packet=(seq,stamp,frame)
        except Exception as exc:
            with self.lock: self.error=str(exc)

    def next(self, *, preview_only=False):
        with self.lock: packet,error=self.packet,self.error
        if error: raise RuntimeError(error)
        if packet is None: return None
        seq,stamp,frame=packet
        if time.monotonic()-stamp>.2:
            # Preview occurs BEFORE any motor port is opened. Window startup
            # may leave an old frame: discard it, never treat it as fresh.
            # Motion/hold callers retain the strict 200ms failure by default.
            if preview_only: return None
            raise RuntimeError('Camera frame older than 200ms; stop hold')
        if seq==self.used: return None
        self.used=seq
        return stamp,frame.copy()

    def detect(self,frame):
        pose=self.tc.step(frame,self.board,self.detector,self.dictionary,self.K,self.dist)
        self.report['last_marker_detection']=dict(self.detector.stats)
        return pose

    def show(self,frame,lines):
        if self.detector is not None: self.detector.draw(frame)
        for i,line in enumerate(lines):
            self.cv.putText(frame,line,(10,25+i*25),self.cv.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),2)
        self.cv.imshow(self.title,frame)

    def record_policy_frame(self,frame,pose,metrics):
        self.policy_evidence.remember(frame,pose,metrics,self.detector)

    def key(self):
        key=self.cv.waitKey(1)&255
        if key in (ord('q'),27): raise KeyboardInterrupt('User stopped camera window')
        return key

    def preview(self):
        self.cv.namedWindow(self.title,self.cv.WINDOW_NORMAL)
        self.cv.resizeWindow(self.title,1100,620)
        last=time.monotonic()
        ready_frames=0
        raw_frame=None
        last_frame_stamp=None
        require_pose=getattr(self,'require_policy_pose',True)
        while True:
            motion.base.check_cancel()
            packet=self.next(preview_only=True)
            valid=not require_pose and last_frame_stamp is not None and time.monotonic()-last_frame_stamp<=.2
            if packet is not None:
                last=time.monotonic()
                stamp,frame=packet
                last_frame_stamp=stamp
                raw_frame=frame.copy()
                if require_pose:
                    pose=self.detect(frame)
                    reason=shadow.pose_rejection(pose,(time.monotonic()-stamp)*1000)
                    ready_frames=ready_frames+1 if reason is None else 0
                    valid=ready_frames>=shadow.ob.HISTORY_LEN
                    status=reason or ('ready' if valid else 'history_warmup')
                    self.report['preview_policy_gate']=dict(status=status, consecutive_valid_frames=ready_frames)
                    lines=['EMPTY HAND. Cube on separate support OUTSIDE finger motion path.',
                        'Show TWO broad cube faces. S saves raw preview for diagnosis.',
                        f'Policy input: {status} | reprojection px: {pose.get("reproj", "missing") if pose else "missing"}',
                        f'Hand + 2 cube markers: {valid}']
                else:
                    valid=time.monotonic()-stamp<=.2
                    self.report['preview_camera_gate']=dict(ready=valid,markers_required=False)
                    lines=['EMPTY HAND. No cube or markers required to start.',
                        'Cube placement is ONLY requested later in the contact trial.',
                        'S saves preview. Check that the hand and cables are clear.']
                self.show(frame,['PREVIEW ONLY | Streaming Webcam | NO MOTOR WRITES YET',*lines,
                    ('SPACE starts after checks + 3s | Q cancel' if getattr(self, 'start_on_space', False)
                     else 'SPACE ready | Q cancel')])
            if time.monotonic()-last>2: raise RuntimeError('No fresh camera preview')
            key=self.key()
            if key==ord('s') and raw_frame is not None:
                path=shadow.ROOT/('vision_preview_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.png')
                try:
                    # OpenCV's Windows filename handling can fail on Korean paths.
                    # Encode in memory and let Python handle the Unicode filename.
                    ok,encoded=self.cv.imencode('.png',raw_frame)
                    if not ok: raise ValueError('PNG encoding failed')
                    path.write_bytes(encoded.tobytes())
                except (OSError,ValueError,self.cv.error) as exc:
                    self.report.setdefault('preview_save_errors',[]).append(str(exc))
                    print(f'Preview save failed: {exc}; preview continues. S retries.',flush=True)
                else:
                    self.report['saved_vision_preview']=str(path)
                    print(f'Saved raw preview: {path}',flush=True)
            if key==32 and valid: return
            time.sleep(.01)

    def close(self):
        self.stop.set()
        if self.thread is not None: self.thread.join(timeout=.5)
        if self.cap is not None: self.cap.release()
        if self.thread is not None: self.thread.join(timeout=.5)
        self.cv.destroyAllWindows()


def fixed_hold(read_checked,camera,engine,report,*,now=time.monotonic,sleep=time.sleep,cancel=motion.base.check_cancel):
    began=last_frame=now()
    samples=report['held_policy_samples']=[]
    report['held_camera_frames']=0
    skips=report['held_policy_skip_counts']={}
    diagnostics=report['held_vision_diagnostics']=[]
    engine.reset()
    while now()-began<10:
        cancel()
        read_start=now()
        state=read_checked()  # Validates fixed goal, caps, current, health, .035rad tracking, total deadline.
        state['read_duration_ms']=(now()-read_start)*1000
        actual=shadow.hw_to_sim(state['position']*shadow.TICK)
        if np.any(actual<shadow.limits.SAFE_LOWERS) or np.any(actual>shadow.limits.SAFE_UPPERS):
            raise RuntimeError('Held joint outside policy range; no policy command sent')
        packet=camera.next()
        if packet is not None:
            stamp,frame=packet
            last_frame=now()
            pose=camera.detect(frame)
            elapsed=(now()-stamp)*1000
            if elapsed>200: raise RuntimeError('Camera/vision age exceeds 200ms during hold')
            result=engine.step(state,pose,elapsed)
            report['held_camera_frames']+=1
            if result is not None: samples.append(dict(result,time_s=now()-began))
            reason=engine.last_skip_reason
            if reason: skips[reason]=skips.get(reason,0)+1
            reproj=pose.get('reproj') if pose else None
            diagnostics.append(dict(time_s=now()-began, skip_reason=reason,
                n_cube_markers=int(pose.get('n_cube_markers',0)) if pose else 0,
                reprojection_px=float(reproj) if reproj is not None and np.isfinite(reproj) else None,
                frame_age_ms=float(elapsed), consecutive_valid_frames=engine.count))
            camera.show(frame,['FIXED START POSE HOLD | POLICY ACTIONS NOT SENT',
                f'{max(0,10-(now()-began)):.1f}s left | inference samples {len(samples)}',
                f'Policy input: {reason or "eligible"}',
                'Do NOT place cube in hand or touch fingers. Q stops.'])
        if now()-last_frame>.2: raise RuntimeError('No new camera frame for 200ms; stop hold')
        camera.key()
        read_checked()  # Check again after inference/display, before the next iteration.
        sleep(.01)
    report['held_inference_percent']=100*len(samples)/report['held_camera_frames'] if report['held_camera_frames'] else 0
    if not samples:
        raise RuntimeError('No eligible policy samples during fixed hold: '+json.dumps(skips))
    report['held_shadow_complete']=True


def main(*, motion_runner=motion, confirmation='HOLD_SHADOW', result_stem='held_policy_shadow', policy_mode=False, contact_mode=False, prepare_only=False, loaded_policy_mode=False, policy_radius_ticks=32, sac_deployment_mode=False):
    if policy_mode or loaded_policy_mode or sac_deployment_mode:
        from hand_motor_mapping import reject_legacy_sac_execution
        try:
            reject_legacy_sac_execution()
        except RuntimeError as exc:
            print('STOP:',str(exc),flush=True)
            return 1
    if sum(map(bool,(policy_mode,contact_mode,prepare_only,loaded_policy_mode)))>1: raise ValueError('Choose one trial mode')
    if sac_deployment_mode and not loaded_policy_mode:
        raise ValueError('SAC deployment requires loaded policy mode')
    policy_duration=20 if sac_deployment_mode else 5
    policy_range='learned joint limits' if sac_deployment_mode else f'+/-{policy_radius_ticks*shadow.TICK:.2f}rad'
    if policy_radius_ticks not in (32,65) or (policy_radius_ticks!=32 and not loaded_policy_mode):
        raise ValueError('Expanded radius requires the explicit loaded-policy trial')
    report=dict(timestamp=datetime.datetime.now().astimezone().isoformat(),status='not_started',
        policy_actions_sent=0,policy_controls_motors=False,hardware_policy_authorized=False,
        cube_location='separate support outside finger path; NOT grasping',
        goal_source='fixed recovery/start-pose plan only',
        timing_note='Host delivery timestamps; NOT synchronized sensor exposure')
    bus=camera=None; opened=False
    if policy_mode:
        report.update(policy_controls_motors=True,hardware_policy_authorized=True,
            goal_source='fixed preparation then bounded policy trial',
            trial_scope='empty hand; +/-0.05rad; 10s; not cube manipulation validation')
    if contact_mode:
        report.update(trial_scope='operator-placed cube, fixed goals, 5s observation; policy output NOT sent',
            contact_mode=True)
    if loaded_policy_mode:
        report.update(loaded_policy_mode=True,policy_controls_motors=True,hardware_policy_authorized=True,
                      goal_source=f'empty preparation, operator placement, {policy_duration}s policy ({policy_range}), operator removal',
                      trial_scope=f'cube on hand; {policy_range}; {policy_duration}s; rate-limited learned policy')
    if sac_deployment_mode:
        import hashlib
        policy_path=shadow.PROJECT/'policy/policy_np.npz'
        report.update(sac_deployment_mode=True,policy_provenance='Distilled PPO teacher + residual SAC (not a standalone SAC actor)',
                      policy_file=str(policy_path),policy_sha256=hashlib.sha256(policy_path.read_bytes()).hexdigest(),
                      target_cube_face=0,initial_pose_source='training DEFAULT_POSE; recorded sequence not executed')
    if prepare_only:
        report.update(prepare_only=True,cube_location='not required; hand empty',
                      trial_scope='empty preparation and 3s hold; no policy or marker tracking')
    try:
        engine=None if prepare_only else shadow.Shadow(shadow.Policy(shadow.PROJECT/'policy/policy_np.npz'),expected_torque=1)
        camera=Camera(report)
        camera.require_policy_pose=not (prepare_only or contact_mode or loaded_policy_mode)
        if prepare_only: camera.title='EMPTY PREPARATION | NO CUBE REQUIRED | Q stops'
        if policy_mode: camera.title='BOUNDED POLICY MOTION | Q stops'
        if contact_mode: camera.title='FIXED CUBE CONTACT CHECK | Q stops'
        if loaded_policy_mode: camera.title=f'CUBE ON HAND | POLICY {policy_duration}s | Q stops'
        camera.start_on_space = confirmation is None
        print('Keep hand EMPTY. No cube required for this preparation test.' if prepare_only else
              'Keep hand EMPTY. Cube is needed only after the PLACE prompt.' if contact_mode or loaded_policy_mode else
              'Keep hand EMPTY. Cube on a separate support outside ALL finger motion paths.',flush=True)
        print(('Preview: no markers required; ' if prepare_only or contact_mode or loaded_policy_mode else 'Preview: show ID40 + 2 cube markers; ')+
              ('SPACE starts movement after checks and a 3s countdown.' if confirmation is None
               else 'SPACE when ready. Then confirm in terminal.'),flush=True)
        camera.preview()
        import dynamixel_sdk as sdk
        bus=motion_runner.Bus(sdk,report)
        if not bus.port.openPort(): raise RuntimeError('Cannot open COM7')
        opened=True
        if not bus.port.setBaudRate(3000000): raise RuntimeError('Cannot set baud')
        initial=bus.preflight(); moves,_,goal=motion_runner.plan(initial['position'])
        print('EMPTY preparation -> fixed 3s hold -> OFF. No cube placement, no policy motion.' if prepare_only else
              f'EMPTY START -> PLACE + SPACE -> POLICY {policy_range} for {policy_duration}s -> REMOVE + SPACE -> OFF.' if loaded_policy_mode else
              'EMPTY START -> PLACE cube on prompt -> fixed 5s observation -> REMOVE cube -> OFF.' if contact_mode else
              'MOVE TO START, then POLICY MOTION +/-0.05rad for 10s.' if policy_mode else
              'ARMED TEST WILL MOVE ALL 16 JOINTS to start pose, then HOLD for 10s.',flush=True)
        print('Current caps by ID:',motion_runner.CAPS.tolist(),flush=True)
        if getattr(motion_runner,'PRESERVE_DEVICE_GAINS',False):
            print('PID: preserve device settings. P/I/D by ID:',
                  {name:initial[name].tolist() for name in ('p_gain','i_gain','d_gain')},flush=True)
        total_limit=getattr(motion_runner,'TOTAL_ON_SECONDS',60)
        print(('Stage timing; no combined ON deadline. ' if total_limit is None else f'Total ON limit {total_limit}s. ')+('Bounded policy goals WILL be sent.' if policy_mode or loaded_policy_mode
              else 'Policy outputs NEVER sent.'),flush=True)
        print('Prepare EMPTY first; place cube ONLY after PLACE prompt. Q stops; cut power for unintended contact/noise.'
              if contact_mode or loaded_policy_mode else
              'Empty hand secured; fingers/cables clear. Q/Ctrl+C stops.' if prepare_only else
              'Empty hand secured; cube separate; fingers/cables clear. Q/Ctrl+C stops; cut power for contact/noise.',flush=True)
        print('Preparation:',motion_runner.__name__,flush=True)
        print('Recovery motor IDs/targets:',[(i+1,target) for i,target in moves],flush=True)
        print('Final target:',goal.tolist(),flush=True)
        if confirmation is not None and input(f'Type {confirmation} to arm: ').strip()!=confirmation:
            report['status']='cancelled_before_writes'; return 0
        report['start_trigger'] = 'preview_space' if confirmation is None else 'terminal_confirmation'
        for seconds in (3, 2, 1):
            print(f'Starting in {seconds}s. Q / Esc / Ctrl+C cancels.',flush=True)
            for _ in range(20): motion.base.check_cancel(); camera.key(); time.sleep(.05)
        fresh=bus.preflight(); fresh_moves,_,fresh_goal=motion_runner.plan(fresh['position'])
        initial_tolerance=4 if policy_mode or contact_mode or prepare_only or loaded_policy_mode else 1
        if (np.any(np.abs(fresh['position']-initial['position'])>initial_tolerance) or
                [i for i,_ in fresh_moves]!=[i for i,_ in moves] or not np.array_equal(goal,fresh_goal)):
            raise RuntimeError('Pose changed before start')
        if camera.next() is None: raise RuntimeError('No fresh camera before enable')
        if prepare_only:
            def hold(reader):
                began=time.monotonic()
                while time.monotonic()-began<3:
                    reader(); time.sleep(.02)
            motion_runner.run_trial(bus,fresh,report,cancel=lambda:(motion.base.check_cancel(),camera.key()),held_check=hold)
            report['status']='empty_preparation_complete'
            print('Empty preparation complete. All torque OFF verified. Next: leap_contact.cmd.',flush=True)
        elif loaded_policy_mode:
            from loaded_policy_trial import run
            from fixed_cube_contact import ContactKeys
            contact_keys=ContactKeys(camera,report)
            motion_runner.run_trial(bus,fresh,report,cancel=contact_keys.poll,
                policy_radius_ticks=policy_radius_ticks,
                policy_joint_limits=sac_deployment_mode,
                active_check=lambda control:run(control,camera,engine.policy,report,input_keys=contact_keys,duration_s=policy_duration))
            report['status']='loaded_policy_trial_complete'
            print('Loaded policy trial complete. Cube removal confirmed; all torque OFF verified.',flush=True)
        elif policy_mode:
            from bounded_policy import run
            motion_runner.run_trial(bus,fresh,report,cancel=lambda:(motion.base.check_cancel(),camera.key()),
                active_check=lambda control:run(control,camera,engine.policy,report))
            report['status']='bounded_policy_motion_complete'
            print('Bounded policy trial complete. All torque OFF verified.',flush=True)
        elif contact_mode:
            from fixed_cube_contact import run, ContactKeys
            contact_keys=ContactKeys(camera,report)
            motion_runner.run_trial(bus,fresh,report,cancel=contact_keys.poll,
                held_check=lambda reader:run(reader,camera,engine,report,
                    cancel=contact_keys.poll,input_keys=contact_keys))
            report['status']='contact_fixed_hold_complete'
            print('Fixed cube contact check complete. All torque OFF verified. Policy output NOT sent.',flush=True)
        else:
            motion_runner.run_trial(bus,fresh,report,cancel=lambda:(motion.base.check_cancel(),camera.key()),
                held_check=lambda reader:fixed_hold(reader,camera,engine,report))
            report['status']='fixed_hold_shadow_complete_not_policy_motion'
            print('Fixed hold + unsent policy check complete. All torque OFF verified.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted')
        print('STOP: '+report['error'],flush=True)
    finally:
        if bus is not None and opened:
            try:
                torque=bus.block(64,1,[('torque',64,1)])['torque']
                report['final_torque_states']=torque.tolist()
                report['all_torque_off_verified']=bool(np.all(torque==0))
            except Exception as exc: report.update(all_torque_off_verified=False,final_check_error=str(exc))
            if not report['all_torque_off_verified']: print('CUT MOTOR POWER NOW: OFF unverified.',flush=True)
            bus.port.closePort()
        if camera is not None:
            try: camera.close()
            except Exception as exc: report['camera_close_error']=str(exc)
        report['tracking_wait_count']=sum(x['event']=='wait' for x in report.get('tracking_wait_events',[]))
        report['tracking_resume_count']=sum(x['event']=='resume' for x in report.get('tracking_wait_events',[]))
        tag=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        if camera is not None and report.get('all_torque_off_verified',False):
            evidence=getattr(camera,'policy_evidence',None)
            if evidence is not None:
                try:
                    evidence.save(shadow.ROOT/f'{result_stem}_vision_{tag}',camera.cv,camera,report)
                except Exception as exc:
                    report.setdefault('policy_evidence_save_errors',[]).append(str(exc))
        for name in (f'{result_stem}_{tag}.json',f'{result_stem}_result.json'):
            path=shadow.ROOT/name; path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(f'Saved: {path}\nPolicy actions sent: {report["policy_actions_sent"]}; '
              f'write attempts: {report.get("policy_write_attempts",0)}. No automatic motion retry.',flush=True)
        if report.get('read_failures'):
            print('Read timeouts recovered:',sum(bool(e.get('recovered')) for e in report['read_failures']),
                  '(fresh reads only; no goal/enable retransmission).',flush=True)
    return 0 if report['status'] in ('loaded_policy_trial_complete','empty_preparation_complete','fixed_hold_shadow_complete_not_policy_motion','bounded_policy_motion_complete','contact_fixed_hold_complete') else 1


if __name__=='__main__': raise SystemExit(main())
