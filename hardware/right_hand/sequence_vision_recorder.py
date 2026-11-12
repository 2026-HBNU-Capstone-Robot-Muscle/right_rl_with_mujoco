"""Diagnostic camera recorder for a known motor sequence.

This module never opens a motor port or sends motor commands. Its camera work
runs on a background thread; the motor loop never waits for marker detection.
"""
import datetime as dt
import json
import threading
import time
from collections import deque
from pathlib import Path
import numpy as np

from leap_vision_check import pose_snapshot, save_image
from vision_diagnostics import frame_metrics, summarize


class SequenceVisionRecorder:
    def __init__(self, root, motor_report, *, camera_factory=None, preview=True,
                 anchor_mode=False, support_photo_mode=False, anchor_min_cube_markers=1):
        self.root = Path(root)
        self.motor_report = motor_report
        self.report = dict(status='starting', frames=[], camera_only_diagnostics=True,
                           motor_port_opened_by_recorder=False)
        if anchor_min_cube_markers not in (1, 2):
            raise ValueError('anchor_min_cube_markers must be 1 or 2')
        self.anchor_min_cube_markers = anchor_min_cube_markers
        self.stop_event = threading.Event()
        self.first_frame = threading.Event()
        self.frame_lock = threading.Lock()
        self.thread = None
        self.error = None
        self.examples = {}
        self.phase_last = {}
        self.recent_frames = deque(maxlen=16)
        self.last_analyzed = None
        self.last_kept_frame = float('-inf')
        self.placement_frame = None
        self.placement_stamp = None
        self.anchor_mode = anchor_mode
        self.support_photo_mode = support_photo_mode
        self.anchor_empty_frame = None
        self.anchor_empty_stamp = None
        self.anchor_preflight_frame = None
        self.anchor_cube_candidate = None
        self.anchor_detector = None
        if anchor_mode:
            import cv2
            from marker_detector import MarkerDetector
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.anchor_detector = MarkerDetector(dictionary, [41], hand_id=40,
                                                  required_cube_count=1)
        self.camera = None
        if camera_factory is None:
            from held_policy_shadow import Camera
            camera_factory = Camera
        # Open and validate the named webcam before any motor port is opened.
        self.camera = camera_factory(self.report)
        try:
            if preview:
                self._preview()
        except BaseException:
            self.camera.close()
            raise
        self.offset = time.time() - time.monotonic()
        self.report['status'] = 'recording'
        self.thread = threading.Thread(target=self._record, name='sequence-vision-recorder', daemon=True)
        self.thread.start()
        deadline = time.monotonic() + 5
        while not self.first_frame.wait(.05):
            if self.error is not None or time.monotonic() >= deadline:
                self.stop_event.set()
                self.thread.join(timeout=1)
                self.camera.close()
                raise RuntimeError(self.error or 'No camera frame within 5s; motor port not opened')

    def _preview(self):
        """Show the actual camera view before any motor port is opened."""
        camera = self.camera
        camera.title = 'LEAP SEQUENCE CAMERA PREVIEW - SPACE starts'
        camera.cv.namedWindow(camera.title, camera.cv.WINDOW_NORMAL)
        camera.cv.resizeWindow(camera.title, 1100, 620)
        print('Camera preview: aim at the HAND, not the ceiling.', flush=True)
        if self.support_photo_mode:
            print('FIRST SPACE starts Stage 0 motion. Keep the hand EMPTY and the cube OUTSIDE camera view until PLACE CUBE.', flush=True)
            print('The camera must show 10 fresh frames without cube markers before FIRST SPACE is accepted.', flush=True)
        else:
            print('Cube stays off the hand until PLACE CUBE.', flush=True)
        if self.anchor_mode:
            print('Before movement, show both hand ID40 and rigid-mount ID41 in this camera.', flush=True)
        print('SPACE in the camera window starts recording and the hand sequence; Q cancels.',flush=True)
        last = time.monotonic()
        cube_free_frames = 0
        cube_ids = []
        try:
            while True:
                packet = camera.next(preview_only=True)
                fresh = False
                if packet is not None:
                    stamp, frame = packet
                    last = time.monotonic()
                    camera.detect(frame)
                    fresh = time.monotonic()-stamp <= .2
                    ids = camera.report.get('last_marker_detection', {}).get('ids', [])
                    anchor_ids = ids
                    cube_ids = [int(i) for i in ids if 0 <= int(i) < 6]
                    if self.support_photo_mode:
                        cube_free_frames = cube_free_frames + 1 if fresh and not cube_ids else 0
                    if self.anchor_mode:
                        gray = camera.cv.cvtColor(frame, camera.cv.COLOR_BGR2GRAY)
                        _, found, _ = self.anchor_detector.detectMarkers(gray)
                        anchor_ids = [] if found is None else [int(i) for i in found.ravel()]
                        camera.cv.line(frame, (0, 80), (frame.shape[1]-1, 80), (0, 255, 255), 2)
                    lines = ['PREVIEW ONLY | NO MOTOR PORT OPEN',
                             'Aim at the hand; cube stays separate until PLACE.',
                             f'Detected marker IDs: {anchor_ids}']
                    if self.support_photo_mode:
                        lines.extend([f'EMPTY HAND / cube OUT of view: {cube_free_frames}/10 frames; cube IDs {cube_ids}',
                                      'FIRST SPACE moves to Stage 0 only when 10/10; Q cancels'])
                    else:
                        lines.append('Leave cube space BELOW yellow line | SPACE starts | Q cancels'
                                     if self.anchor_mode else 'SPACE starts after motor checks + 3s | Q cancels')
                    camera.show(frame, lines)
                if time.monotonic()-last > 2:
                    raise RuntimeError('No fresh camera preview frame for 2s; motor port not opened')
                key = camera.key()
                if (key == 32 and fresh
                        and (not self.anchor_mode or (40 in anchor_ids and 41 in anchor_ids))
                        and (not self.support_photo_mode or cube_free_frames >= 10)):
                    return
                time.sleep(.01)
        finally:
            camera.cv.destroyWindow(camera.title)

    def _record(self):
        last_frame = time.monotonic()
        try:
            while not self.stop_event.is_set():
                packet = self.camera.next(preview_only=True)
                if packet is None:
                    if time.monotonic() - last_frame > (2 if self.first_frame.is_set() else 5):
                        raise RuntimeError('No fresh camera frame for 2s')
                    self.stop_event.wait(.005)
                    continue
                stamp, frame = packet
                last_frame = time.monotonic()
                pose = self.camera.detect(frame)
                metrics = frame_metrics(pose, (time.monotonic()-stamp)*1000,
                                        self.report.get('last_marker_detection', {}))
                phase = self.motor_report.get('phase', 'preflight')
                enabled_at = self.motor_report.get('enabled_at')
                row = dict(captured_at_unix_s=float(stamp+self.offset),
                           motor_t_s=float(stamp-enabled_at) if enabled_at is not None else None,
                           phase=phase, **metrics, pose=pose_snapshot(pose))
                if self.anchor_mode:
                    gray = self.camera.cv.cvtColor(frame, self.camera.cv.COLOR_BGR2GRAY)
                    _, anchor_ids, _ = self.anchor_detector.detectMarkers(gray)
                    row['anchor_detected_ids'] = ([] if anchor_ids is None else
                                                  [int(i) for i in anchor_ids.ravel()])
                    if phase == 'stage_0' and 40 in row['anchor_detected_ids'] and 41 in row['anchor_detected_ids']:
                        with self.frame_lock:
                            self.anchor_empty_frame = frame.copy()
                            self.anchor_empty_stamp = stamp
                    if phase == 'preflight' and 40 in row['anchor_detected_ids'] and 41 in row['anchor_detected_ids']:
                        with self.frame_lock:
                            self.anchor_preflight_frame = frame.copy()
                    cube_error = None if pose is None else pose.get('reproj')
                    if (phase == 'place_cube' and 41 in row['anchor_detected_ids']
                            and len({i for i in row['detected_ids'] if 0 <= i < 6}) >= self.anchor_min_cube_markers
                            and pose is not None and 't_cube' in pose
                            and cube_error is not None and np.isfinite(cube_error)
                            and cube_error <= 3.0):
                        with self.frame_lock:
                            self.anchor_cube_candidate = (stamp, frame.copy(), row)
                self.report['frames'].append(row)
                with self.frame_lock:
                    self.last_analyzed = (stamp, frame.copy(), row)
                key = (phase, 'eligible' if metrics['reason'] is None else 'not_ready')
                self.examples.setdefault(key, frame.copy())
                # Keep one final frame per phase. The existing examples above
                # capture phase entry only and can miss a face change mid-stage.
                self.phase_last[phase] = (stamp, frame.copy(), row)
                if stamp-self.last_kept_frame >= .4:
                    with self.frame_lock:
                        self.recent_frames.append((stamp, frame.copy()))
                    self.last_kept_frame = stamp
                self.first_frame.set()
        except Exception as exc:
            self.error = str(exc)
            self.report['status'] = 'camera_stopped'

    def mark_placement(self, *, require_two_cube=False, require_anchor_cube=False,
                       require_support_photo=False):
        """Remember the analyzed frame nearest PLACE; optionally verify its IDs."""
        with self.frame_lock:
            if require_anchor_cube:
                if self.anchor_empty_frame is None:
                    raise RuntimeError('No stage-0 image with ID40 and ID41 together')
                if self.anchor_cube_candidate is None:
                    raise RuntimeError('No ID41 + cube marker image yet')
                stamp, frame, row = self.anchor_cube_candidate
                age = time.monotonic() - stamp
                if self.anchor_min_cube_markers == 2 and age > .5:
                    raise RuntimeError('Fresh ID41 + 2 cube markers required in one frame')
                row = dict(row, capture_age_s=float(age), capture_stale=bool(age > 3.0))
                self.placement_stamp = stamp
                self.placement_frame = frame.copy()
                return row
            if self.last_analyzed is not None:
                stamp, frame, row = self.last_analyzed
                if require_support_photo and time.monotonic() - stamp > 1.0:
                    raise RuntimeError('Fresh camera frame required for support photo')
                if require_two_cube:
                    ids = row['detected_ids']
                    if time.monotonic() - stamp > .5 or 40 not in ids or sum(i < 40 for i in ids) < 2:
                        raise RuntimeError(f'Fresh ID40 + 2 cube markers required in one frame; detected {ids}')
                self.placement_stamp = stamp
                self.placement_frame = frame.copy()
                return row
        if require_two_cube or require_anchor_cube or require_support_photo:
            raise RuntimeError('No analyzed camera frame available')
        return None

    def check_anchor_rigidity(self):
        """Reject a changing ID40-to-ID41 relation before asking for the cube."""
        if not self.anchor_mode:
            return None
        with self.frame_lock:
            before = None if self.anchor_preflight_frame is None else self.anchor_preflight_frame.copy()
            after = None if self.anchor_empty_frame is None else self.anchor_empty_frame.copy()
        if before is None or after is None:
            raise RuntimeError('ID40 + ID41 were not both visible before and after stage 0; keep cube off')
        from sac_registration_from_anchor import marker_pose
        dictionary = self.camera.cv.aruco.getPredefinedDictionary(self.camera.cv.aruco.DICT_4X4_50)
        from marker_detector import MarkerDetector
        detector = MarkerDetector(dictionary, [41], hand_id=40, required_cube_count=1)
        with np.load(self.root / 'camera1_720.npz') as cal:
            K, distortion = cal['K'], cal['dist']
        poses = []
        for frame in (before, after):
            corners, found, _ = detector.detectMarkers(self.camera.cv.cvtColor(frame, self.camera.cv.COLOR_BGR2GRAY))
            ids = [] if found is None else [int(i) for i in found.ravel()]
            R_h, t_h, _ = marker_pose(corners, ids, 40, .0282, K, distortion)
            R_a, t_a, _ = marker_pose(corners, ids, 41, .030, K, distortion)
            observed40 = next(np.asarray(c).reshape(4, 2) for c, i in zip(corners, ids) if i == 40)
            poses.append((R_a, t_a, R_h, t_h, observed40))
        R_a0, t_a0, R_h0, t_h0, _ = poses[0]
        R_a1, t_a1, _, _, observed40 = poses[1]
        R_anchor_hand = R_a0.T @ R_h0
        t_anchor_hand = R_a0.T @ (t_h0 - t_a0)
        predicted_R = R_a1 @ R_anchor_hand
        predicted_t = t_a1 + R_a1 @ t_anchor_hand
        s = .0282 / 2
        obj = np.asarray([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
        rvec, _ = self.camera.cv.Rodrigues(predicted_R)
        projected, _ = self.camera.cv.projectPoints(obj, rvec, predicted_t, K, distortion)
        error_px = float(np.linalg.norm(projected.reshape(4, 2) - observed40, axis=1).mean())
        self.report['anchor_rigid_projection_error_px'] = error_px
        print(f'ID40/ID41 rigid-reference image check: {error_px:.1f}px', flush=True)
        if error_px > 10:
            raise RuntimeError(f'ID41 reference moved/warped relative to ID40 ({error_px:.1f}px); '
                               'keep cube off; torque OFF now')
        return error_px

    def finish(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3)
        if self.thread is not None and self.thread.is_alive():
            self.error = self.error or 'Camera diagnosis thread did not stop within 3s'
        if self.camera is not None:
            try:
                self.camera.close()
            except Exception as exc:
                self.error = self.error or f'Camera close failed: {exc}'
        self.report['status'] = 'complete' if self.error is None else 'camera_stopped'
        if self.error is not None:
            self.report['error'] = self.error
        rows = self.report['frames']
        self.report['summary'] = summarize(rows)
        self.report['by_phase'] = {
            phase: summarize([row for row in rows if row['phase'] == phase])
            for phase in dict.fromkeys(row['phase'] for row in rows)
        }
        stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        folder = self.root / f'sequence_vision_{stamp}'
        folder.mkdir(exist_ok=True)
        saved = []
        for (phase, kind), frame in self.examples.items():
            path = folder / f'{phase}_{kind}.png'
            try:
                save_image(self.camera.cv, path, frame)
                saved.append(str(path))
            except Exception as exc:
                self.report.setdefault('save_errors', []).append(str(exc))
        phase_last_metadata = {}
        for phase, (last_stamp, frame, row) in self.phase_last.items():
            if not phase.startswith('stage_') and phase != 'place_cube':
                continue
            path = folder / f'{phase}_end.png'
            try:
                save_image(self.camera.cv, path, frame)
                saved.append(str(path))
                phase_last_metadata[phase] = dict(
                    image=str(path), captured_at_unix_s=float(last_stamp+self.offset),
                    motor_t_s=row.get('motor_t_s'), detected_ids=row.get('detected_ids', []))
            except Exception as exc:
                self.report.setdefault('save_errors', []).append(str(exc))
        self.report['phase_end_images'] = phase_last_metadata
        if self.placement_frame is not None:
            path = folder / 'placement_at_space.png'
            try:
                save_image(self.camera.cv, path, self.placement_frame)
                saved.append(str(path))
                self.report['placement_image_captured_at_unix_s'] = float(self.placement_stamp+self.offset)
            except Exception as exc:
                self.report.setdefault('save_errors', []).append(str(exc))
        if self.anchor_empty_frame is not None:
            path = folder / 'anchor_empty_stage0.png'
            try:
                save_image(self.camera.cv, path, self.anchor_empty_frame)
                saved.append(str(path))
                self.report['anchor_empty_captured_at_unix_s'] = float(self.anchor_empty_stamp+self.offset)
            except Exception as exc:
                self.report.setdefault('save_errors', []).append(str(exc))
        if self.anchor_preflight_frame is not None:
            path = folder / 'anchor_empty_preflight.png'
            try:
                save_image(self.camera.cv, path, self.anchor_preflight_frame)
                saved.append(str(path))
            except Exception as exc:
                self.report.setdefault('save_errors', []).append(str(exc))
        if self.anchor_cube_candidate is not None:
            candidate_stamp, candidate_frame, candidate_row = self.anchor_cube_candidate
            path = folder / 'anchor_cube_candidate.png'
            try:
                save_image(self.camera.cv, path, candidate_frame)
                saved.append(str(path))
                self.report['anchor_cube_candidate_at_unix_s'] = float(candidate_stamp+self.offset)
                self.report['anchor_cube_candidate_ids'] = candidate_row['detected_ids']
            except Exception as exc:
                self.report.setdefault('save_errors', []).append(str(exc))
        samples = self.motor_report.get('samples', [])
        enabled_at = self.motor_report.get('enabled_at')
        if (self.motor_report.get('status') == 'stopped' and samples
                and enabled_at is not None and self.recent_frames):
            failure_stamp = enabled_at + samples[-1]['t']
            near_stamp, near_frame = min(self.recent_frames,
                                         key=lambda pair: abs(pair[0]-failure_stamp))
            path = folder / 'failure_nearest.png'
            try:
                save_image(self.camera.cv, path, near_frame)
                saved.append(str(path))
                self.report['failure_image_offset_s'] = float(near_stamp-failure_stamp)
            except Exception as exc:
                self.report.setdefault('save_errors', []).append(str(exc))
        self.report['saved_images'] = saved
        self.report['folder'] = str(folder)
        result = folder / 'result.json'
        result.write_text(json.dumps(self.report, ensure_ascii=False, indent=2), encoding='utf-8')
        return dict(status=self.report['status'], folder=str(folder), result=str(result),
                    summary=self.report['summary'], by_phase=self.report['by_phase'],
                    error=self.report.get('error'), save_errors=self.report.get('save_errors', []))
