"""Fixed-goal cube loading check. Only a checked READ callback is accepted.

Preparation and final OFF belong to the outer runner. Policy output is logged,
never sent. The operator confirms placement and removal with SPACE.
"""
import time
import numpy as np
import encoder_policy_shadow as shadow

PLACE_SECONDS, OBSERVE_SECONDS, REMOVE_SECONDS = 20., 5., 8.
MIN_PLACE_SECONDS, TIME_MARGIN_SECONDS = 10., 2.


class ContactKeys:
    """Keep SPACE across nested health reads; Q/Esc/Ctrl+C interrupt immediately."""
    def __init__(self, camera, report, console=None):
        if console is None:
            import msvcrt
            console = msvcrt
        self.camera, self.report, self.console = camera, report, console
        self.pending = None
        self.extended = False
        report['contact_input_sources'] = ['camera_window', 'terminal']
        report['contact_key_events'] = []

    def accept(self, key, source):
        if key in (ord('q'), ord('Q'), 27, 3): raise KeyboardInterrupt('User stop: '+source)
        phase = self.report.get('contact_phase')
        accepting=(self.report.get('phase')=='fixed_start_pose_check' or
                   (self.report.get('loaded_policy_mode',False) and self.report.get('phase')=='bounded_policy_motion'))
        if key == 32 and accepting and phase in ('place','remove'):
            if self.pending is None or self.pending[0] != phase:
                self.pending = (phase, source)

    def poll(self):
        # Bounded draining; do not let a stream of console input delay health reads.
        for _ in range(32):
            if not self.console.kbhit(): break
            char = self.console.getwch()
            if self.extended:
                self.extended = False
                continue
            if char in ('\x00','\xe0'):
                self.extended = True
                continue
            self.accept(ord(char), 'terminal')
        self.accept(self.camera.key(), 'camera_window')

    def key(self):
        self.poll()
        pending, self.pending = self.pending, None
        if pending is not None and pending[0] == self.report.get('contact_phase'):
            self.report['contact_key_events'].append(dict(phase=pending[0], source=pending[1], key='SPACE'))
            return 32
        return 255


def placement_seconds(used, total_on_seconds=60):
    if total_on_seconds not in (60,90): raise RuntimeError('Unsupported contact ON time limit')
    if used is None or not np.isfinite(float(used)) or float(used)<0:
        raise RuntimeError('Missing/invalid ON time; keep hand empty')
    available = np.floor(total_on_seconds-float(used)-OBSERVE_SECONDS-REMOVE_SECONDS-TIME_MARGIN_SECONDS)
    if available<MIN_PLACE_SECONDS:
        raise RuntimeError('Insufficient ON time for at least 10s placement plus observation/removal; keep hand empty')
    return min(PLACE_SECONDS, float(available))


def run(read_checked, camera, engine, report, *, now=time.monotonic,
        sleep=time.sleep, cancel=lambda: None, input_keys=None):
    baseline = read_checked()
    used = report.get('samples', [{}])[-1].get('t')
    total_on_seconds = report.get('total_on_limit_s',60)
    operator_paced = report.get('operator_paced_contact',False)
    place_seconds = None if operator_paced else placement_seconds(used,total_on_seconds)
    remove_seconds = None if operator_paced else REMOVE_SECONDS
    report.update(contact_phase='place', cube_placement_confirmed=False,
        cube_removal_confirmed=False, contact_samples=[], contact_inferences=[],
        contact_skip_counts={}, contact_fixed_goal_raw=baseline['goal'].tolist(),
        contact_baseline_current_raw=baseline['current'].tolist(),
        contact_time_limits_s=dict(place=place_seconds, observe=OBSERVE_SECONDS,
                                   remove=remove_seconds, margin=TIME_MARGIN_SECONDS),
        contact_on_seconds_at_start=float(used))
    engine.reset()
    phase = 'place'
    began = phase_start = last_frame = now()
    last_print = -float('inf')
    pending_error = None
    print('PLACE CUBE NOW: goals fixed. Gently place on the prepared hand; do not force a fit.', flush=True)
    if operator_paced:
        print('Placement/removal wait for SPACE; observation 5s. Health checks remain active; Q turns torque OFF.',flush=True)
    else:
        print(f'Placement window: {place_seconds:g}s; observation 5s; removal 8s; total ON limit {total_on_seconds:g}s.', flush=True)
    print('Hands clear, then SPACE in CAMERA or TERMINAL to measure 5s. Q = immediate torque OFF.', flush=True)
    while True:
        cancel()
        read_start = now()
        state = read_checked()
        state['read_duration_ms'] = (now()-read_start)*1000
        if not np.array_equal(state['goal'], baseline['goal']):
            raise RuntimeError('Fixed contact goal changed')
        q = shadow.hw_to_sim(state['position']*shadow.TICK)
        if np.any(q<shadow.limits.SAFE_LOWERS) or np.any(q>shadow.limits.SAFE_UPPERS):
            raise RuntimeError('Contact joint outside policy limits')
        packet = camera.next(preview_only=True)
        status = 'waiting_for_frame'
        if packet is not None:
            stamp, frame = packet
            last_frame = now()
            pose = camera.detect(frame)
            age_ms = (now()-stamp)*1000
            reason = shadow.pose_rejection(pose, age_ms)
            status = reason or 'vision_ready'
            if phase == 'observe':
                result = engine.step(state, pose, age_ms)
                if result is not None:
                    report['contact_inferences'].append(dict(result, time_s=now()-began))
                if reason:
                    skips = report['contact_skip_counts']
                    skips[reason] = skips.get(reason, 0)+1
                raw_current = np.asarray(state['current'], dtype=np.int64)
                current = np.where(raw_current>=32768, raw_current-65536, raw_current)
                record = dict(time_s=now()-began, frame_age_ms=age_ms, reason=reason,
                    current_ma=current.tolist(), position_raw=state['position'].tolist(),
                    tracking_error_rad=((state['position']-baseline['goal'])*shadow.TICK).tolist(),
                    voltage_raw=state['voltage'].tolist(), temperature_c=state['temperature'].tolist(),
                    detection=dict(report.get('last_marker_detection', {})))
                if reason is None:
                    record.update(palm_minus_cube_m=np.asarray(pose['palm_minus_cube']).tolist(),
                        cube_quat_world=np.asarray(pose['cube_quat_world']).tolist(),
                        reprojection_px=float(pose['reproj']))
                report['contact_samples'].append(record)
            prompts = dict(place='PLACE cube; hands clear; SPACE starts 5s measurement',
                observe='OBSERVING 5s | keep hands clear | policy output NOT sent',
                remove='REMOVE cube now; SPACE confirms removed and turns torque OFF')
            limit = dict(place=place_seconds, observe=OBSERVE_SECONDS, remove=remove_seconds)[phase]
            timing = 'Waiting for SPACE' if limit is None else f'{max(0,limit-(now()-phase_start)):.1f}s'
            camera.show(frame, ['FIXED CUBE CONTACT CHECK | Q stops immediately', prompts[phase],
                f'{timing} | {status}',
                'SPACE works in CAMERA or TERMINAL. Goals fixed.'])
        elif now()-last_frame>.2:
            engine.reset()
        # The outer runner supplies all health and total-time checks on both reads.
        read_checked()
        if now()-last_frame>2:
            raise RuntimeError('No fresh camera for 2s during contact check')
        key = input_keys.key() if input_keys is not None else camera.key()
        elapsed = now()-phase_start
        if now()-last_print>=1:
            print(f'contact: {phase}; {status}; elapsed {elapsed:.1f}s', flush=True)
            last_print = now()
        if phase == 'place':
            if place_seconds is not None and elapsed>=place_seconds:
                pending_error = 'Cube placement was not confirmed'
                phase, phase_start = 'remove', now()
                print('REMOVE CUBE if placed; SPACE confirms removed. Torque OFF in at most 8s.', flush=True)
            elif key == 32:
                report.update(cube_placement_confirmed=True, cube_location='operator confirmed on prepared hand')
                phase, phase_start = 'observe', now()
                engine.reset()
                print('OBSERVING: hands clear; fixed goals for 5s.', flush=True)
        elif phase == 'observe' and elapsed>=OBSERVE_SECONDS:
            report['contact_observation_complete'] = True
            phase, phase_start = 'remove', now()
            print('REMOVE CUBE NOW; SPACE confirms removed and turns torque OFF. '+
                  ('Waiting for SPACE; Q stops immediately.' if operator_paced else 'Torque OFF in at most 8s.'),flush=True)
        elif phase == 'remove':
            if key == 32:
                report.update(cube_removal_confirmed=True, cube_location='operator confirmed removed')
                break
            if remove_seconds is not None and elapsed>=remove_seconds:
                pending_error = 'Cube removal not confirmed within 8s; torque OFF now'
                break
        report['contact_phase'] = phase
        sleep(.02)
    report['contact_phase'] = 'finished'
    rows = report['contact_samples']
    good = [r for r in rows if r['reason'] is None]
    report['contact_summary'] = dict(frames=len(rows), eligible_frames=len(good),
        unsent_policy_inferences=len(report['contact_inferences']), policy_actions_sent=0,
        peak_abs_current_ma_by_id=np.max(np.abs([r['current_ma'] for r in rows]),axis=0).tolist() if rows else None,
        max_tracking_error_rad=float(np.max(np.abs([r['tracking_error_rad'] for r in rows]))) if rows else None,
        median_palm_minus_cube_m=np.median([r['palm_minus_cube_m'] for r in good],axis=0).tolist() if good else None,
        position_span_mm=(np.ptp([r['palm_minus_cube_m'] for r in good],axis=0)*1000).tolist() if len(good)>=2 else None)
    if pending_error: raise RuntimeError(pending_error)
    if not report['contact_inferences']:
        raise RuntimeError('No eligible contact policy samples; goals stayed fixed')
    report['contact_fixed_hold_complete'] = True
