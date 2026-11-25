"""Review saved contact frames and encoders; never open a camera or motor port."""
import hashlib
import json
import os
from pathlib import Path

import cv2 as cv
import numpy as np

ROOT = Path(__file__).resolve().parent
FOLDER = ROOT / 'cube_grasp_trial_20261023_044607_477240'


def main():
    raw = (FOLDER / 'result.json').read_bytes()
    result = json.loads(raw)
    assert result['status'] == 'cube_grasp_approach_trial_complete'
    assert result['all_torque_off_verified'] and result['settings_restored']
    assert result['cube_removal_confirmed']
    rows = result['samples']
    times = np.array([s['t'] for s in rows], dtype=float)
    assert np.all(np.diff(times) >= 0)
    frames = json.loads((FOLDER / 'camera' / 'frames.json').read_bytes())['frames']
    contact = [f for f in frames if f['phase'] == 'remove_cube']
    frame_times = np.array([f['camera_read_monotonic'] for f in contact])
    assert len(contact) > 2 and np.all(np.diff(frame_times) > 0)
    interval = float(np.median(np.diff(frame_times)))
    fps = 1 / interval

    def load(frame):
        image = cv.imdecode(np.frombuffer((FOLDER / 'camera' / frame['image']).read_bytes(), np.uint8), cv.IMREAD_COLOR)
        if image is None:
            raise RuntimeError('Saved image cannot be decoded: ' + frame['image'])
        return image

    first = load(contact[0])
    height, width = first.shape[:2]
    clip_name = 'recorded_contact_view.mp4'
    old_cwd = Path.cwd()
    writer = None
    try:
        os.chdir(FOLDER)
        writer = cv.VideoWriter(clip_name, cv.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
        if not writer.isOpened():
            raise RuntimeError('Cannot create saved-frame review video')
        for frame in contact:
            image = load(frame)
            if image.shape != first.shape:
                raise RuntimeError('Saved frame size changed')
            writer.write(image)
        writer.release()
        writer = None
        capture = cv.VideoCapture(clip_name)
        try:
            assert capture.isOpened()
            exported_count = int(capture.get(cv.CAP_PROP_FRAME_COUNT))
            assert exported_count == len(contact)
            decoded, _ = capture.read()
            assert decoded
        finally:
            capture.release()
    finally:
        if writer is not None:
            writer.release()
        os.chdir(old_cwd)

    review = dict(
        source=FOLDER.name,
        source_result_sha256=hashlib.sha256(raw).hexdigest(),
        status='grasp_and_contact_recording_complete',
        all_torque_off_verified=True, settings_restored=True, cube_removal_confirmed=True,
        completed_loaded_approach_samples=sum(s['phase'] == 'stage_3_loaded_approach' for s in rows),
        camera_frame_count=result['camera_recording']['frame_count'],
        blank_frame_count=result['camera_recording']['image_signal_blank_frames'],
        contact_frame_count=len(contact),
        contact_capture_span_seconds=float(frame_times[-1] - frame_times[0]),
        max_controller_sample_gap_ms=float(np.max(np.diff(times)) * 1000),
        max_read_ms=max(s.get('read_ms', 0) for s in rows),
        watchdog_fault_samples=sum(255 in s.get('watchdog', []) for s in rows),
        min_voltage_v=min(min(s['voltage_v']) for s in rows),
        peak_current_ma=max(max(abs(c) for c in s['current_ma']) for s in rows),
        max_tracking_rad=max(max(s['tracking_rad']) for s in rows),
        review_video=clip_name,
        review_video_frame_count=exported_count,
        video_timing='Original full-size saved frames at their median capture interval; approximate timing, source timestamps preserved in camera/frames.json.',
        visual_observation='The front shows the thumb pad against the orange cube face. Side views of the yellow/green faces and opposite fingers were saved, including blurred and cropped portions. A calibrated pinch/contact force or metric fingertip gap cannot be established from these images alone.',
        physical_rotation_tested=False,
        policy_used=result['policy_used'],
        repeat_recording_requested=False,
        motor_targets_or_limits_changed=False,
        agent_opened_devices=False,
    )
    (FOLDER / 'contact_capture_review.json').write_text(json.dumps(review, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(review, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
