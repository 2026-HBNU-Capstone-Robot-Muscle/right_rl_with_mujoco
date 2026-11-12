"""Capture cube placement on the new, checked start; no flip/policy motion."""
import time
import numpy as np
import sequence_flip_checked as checked


class PlacementPreview(checked.SupportPhotoPreview):
    def __init__(self, recorder):
        super().__init__(recorder)
        self.title = 'LEAP NEW SAC START - SPACE photo, then remove cube'

    def show(self):
        with self.recorder.frame_lock:
            latest = self.recorder.last_analyzed
            if latest is None:
                return False
            stamp, frame, row = latest
            frame = frame.copy()
        if time.monotonic()-stamp > .5:
            raise RuntimeError('Camera frame stale; no placement capture')
        if not self.created:
            self.cv.namedWindow(self.title, self.cv.WINDOW_NORMAL)
            self.cv.resizeWindow(self.title, 1100, 620)
            self.created = True
        ids = sorted(set(row.get('detected_ids', [])) | set(row.get('anchor_detected_ids', [])))
        for i, line in enumerate(('NEW START: place cube, GREEN up / ORANGE toward camera',
                                  'Hands clear: SPACE saves photo; fingers HOLD until removal',
                                  'Keep the WHOLE cube inside frame, with space above the TOP edge',
                                  f'IDs {ids} | show ID41 and visible cube face; Q stops')):
            self.cv.putText(frame,line,(10,25+25*i),self.cv.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),2)
        self.cv.imshow(self.title,frame)
        return self._key()


def remember_final_empty(recorder, report, began):
    """Save a fresh final-pose photo even when fingers cover palm ID40.

    This records evidence only. It cannot certify a camera/model registration.
    The earlier qualified ID40+41 reference is deliberately preserved separately.
    """
    with recorder.frame_lock:
        latest = recorder.last_analyzed
        if latest is None:
            return False
        stamp, frame, row = latest
        if stamp < began or time.monotonic()-stamp > .5:
            return False
        ids = sorted(set(row.get('anchor_detected_ids', [])))
        recorder.examples[('revised_start_final_empty', 'diagnostic')] = frame.copy()
        report['final_empty_reference_camera_ids'] = ids
        report['final_empty_reference'] = dict(image='revised_start_final_empty_diagnostic.png',
            captured_at_monotonic_s=float(stamp), hand_marker_visible=40 in ids,
            anchor_visible=41 in ids, registration_verified=False,
            reference_source='earlier ID40+ID41 image retained; final image is diagnostic only')
        report['registration_verified'] = False
        return True


def capture_placement(bus, command, report, recorder):
    import msvcrt
    report['phase'] = 'revised_start_empty_reference'
    began = time.monotonic()
    while True:
        checked.sample(bus,command,report,report['phase'])
        if remember_final_empty(recorder,report,began):
            break
        if time.monotonic()-began > 5:
            raise RuntimeError('No fresh camera frame after new pose; keep cube off')
        time.sleep(.02)
    print('Final empty photo saved. ID40 may be covered by fingers; no marker gate after preparation.',flush=True)
    print('Earlier ID40+ID41 reference retained. Photo capture does not certify 3D registration.',flush=True)
    report['phase'] = 'place_cube'
    print('PLACE CUBE NOW on the new pose: GREEN up, ORANGE toward camera. Do not force a fit.',flush=True)
    print('Hands clear, then SPACE in CAMERA or TERMINAL saves photo. No flip will run.',flush=True)
    print('After photo, REMOVE cube and SPACE in TERMINAL turns torque OFF. Q stops.',flush=True)
    # Clear keys held while preparing so they cannot confirm placement.
    while msvcrt.kbhit(): msvcrt.getwch()
    preview = PlacementPreview(recorder)
    began = time.monotonic()
    try:
        while True:
            checked.sample(bus,command,report,'place_cube')
            accepted = preview.show()
            if msvcrt.kbhit():
                key = msvcrt.getwch().lower()
                if key in ('q','\x1b'): raise KeyboardInterrupt('User stop')
                accepted |= key == ' '
            if accepted:
                row = recorder.mark_placement(require_support_photo=True)
                report['placement_snapshot'] = row
                report['placement_joint_ticks'] = report['samples'][-1]['actual']
                report['placement_goal_ticks'] = np.asarray(command).tolist()
                report['registration_verified'] = False
                break
            if time.monotonic()-began > 90:
                raise RuntimeError('Placement capture exceeded 90s; torque OFF')
            time.sleep(.02)
    finally:
        preview.close()
    print('Photo saved for offline pose comparison; no SAC action sent.',flush=True)
    while msvcrt.kbhit(): msvcrt.getwch()
    checked.wait_for_removal(bus,command,report)


if __name__ == '__main__':
    from leap_sac_start_empty import main
    raise SystemExit(main(capture=True))
