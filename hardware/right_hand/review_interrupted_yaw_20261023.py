"""Stored-only review: exclude all images after the last healthy ON read."""
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT = Path(__file__).resolve().parent
FOLDER = ROOT / 'grasped_yaw_trial_20261023_215230_227769'
BASE = 'stage_4_fixed_grasp_observation'
TURN = 'stage_5_grasped_small_yaw'


def load(row):
    image = cv.imdecode(np.frombuffer((FOLDER / 'camera' / row['image']).read_bytes(), np.uint8), cv.IMREAD_COLOR)
    if image is None or image.shape != (720, 1280, 3):
        raise ValueError('Invalid saved frame')
    return image


def main():
    blob = (FOLDER / 'result.json').read_bytes()
    r = json.loads(blob)
    samples = r['samples']
    baseline = [s for s in samples if s['phase'] == BASE]
    turn = [s for s in samples if s['phase'] == TURN]
    cutoff = r['enabled_at'] + samples[-1]['t']
    frames = json.loads((FOLDER / 'camera/frames.json').read_bytes())['frames']
    before = [f for f in frames if f['phase'] == BASE and f['camera_read_monotonic'] <= cutoff]
    during = [f for f in frames if f['phase'] == TURN and f['camera_read_monotonic'] <= cutoff]
    assert len(baseline) == 60 and len(turn) == 19 and before and during
    rows = before[-2:] + during[::max(1, len(during)//3)]
    rows = (rows + [during[-1]])[:6]
    panels = []
    for row in rows:
        image = load(row)
        panel = cv.resize(image, (640, 360))
        label = ('BASE' if row['phase'] == BASE else 'TURN') + f" {row['camera_read_monotonic'] - r['enabled_at']:.3f}s"
        cv.rectangle(panel, (0, 0), (640, 30), (0, 0, 0), -1)
        cv.putText(panel, label, (8, 22), cv.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
        panels.append(panel)
    while len(panels) < 6:
        panels.append(np.zeros_like(panels[0]))
    sheet = np.vstack([np.hstack(panels[:3]), np.hstack(panels[3:6])])
    ok, encoded = cv.imencode('.jpg', sheet, [cv.IMWRITE_JPEG_QUALITY, 95])
    assert ok
    (FOLDER / 'before_interrupted_turn_contact_sheet.jpg').write_bytes(encoded.tobytes())
    selected = [f for f in frames if before[0]['camera_read_monotonic'] <= f['camera_read_monotonic'] <= cutoff]
    stamps = np.array([f['camera_read_monotonic'] for f in selected])
    timeline = np.arange(stamps[0], stamps[-1], 1/15.)
    previous = Path.cwd()
    try:
        os.chdir(FOLDER)
        name = 'grasp_then_interrupted_yaw.mp4'
        writer = cv.VideoWriter(name, cv.VideoWriter_fourcc(*'mp4v'), 15., (1280, 720))
        assert writer.isOpened()
        try:
            for stamp in timeline:
                index = max(0, int(np.searchsorted(stamps, stamp, side='right') - 1))
                writer.write(load(selected[index]))
        finally:
            writer.release()
        reader = cv.VideoCapture(name)  # File decoder only, never a device index.
        try:
            assert reader.isOpened()
            count = int(reader.get(cv.CAP_PROP_FRAME_COUNT))
            ok, decoded = reader.read()
            assert ok and decoded.shape == (720, 1280, 3) and count == len(timeline)
        finally:
            reader.release()
    finally:
        os.chdir(previous)
    report = dict(source=FOLDER.name, source_sha256=hashlib.sha256(blob).hexdigest(),
        status='incomplete_turn_stopped_for_two_failed_fresh_sync_reads',
        failed_phase=r['failed_phase'], error=r['error'],
        phases=dict(Counter(s['phase'] for s in samples)),
        before_fixed_samples=len(baseline), turn_samples_before_stop=len(turn), planned_turn_samples=90,
        healthy_turn_sample_span_s=turn[-1]['t']-turn[0]['t'],
        before_removal_cutoff_camera_monotonic=cutoff,
        actual_joint_change_from_baseline_ticks=(np.array(turn[-1]['actual'])-np.median([s['actual'] for s in baseline[-20:]], axis=0)).tolist(),
        read_failures=r.get('read_failures', []),
        minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
        maximum_absolute_current_ma=max(max(abs(x) for x in s['current_ma']) for s in samples),
        maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),
        all_torque_off_verified=r['all_torque_off_verified'], settings_restored=r['settings_restored'],
        camera_frames=len(frames), camera_blank_frames=r['camera_recording']['image_signal_blank_frames'],
        camera_original_settings=r['camera_exposure_worker']['original_settings'],
        camera_applied_settings=r['camera_exposure_worker']['applied']['readback'],
        camera_restored=r['camera_exposure_restoration_verified'],
        physical_rotation_verified=False, full_turn_complete=False, live_sac=False,
        video='grasp_then_interrupted_yaw.mp4', video_duration_s=len(timeline)/15.,
        source_images_modified=False, real_devices_opened=False,
        visual_verdict='pending_visual_review_of_pre_fault_frames')
    (FOLDER / 'interrupted_yaw_review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
