"""Summarize the operator's three saved read-only checks; never open devices."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FOLDERS = (
    'camera_readonly_check_20261023_043927_771511',
    'camera_readonly_check_20261023_044004_522148',
    'camera_readonly_check_20261023_044029_674824',
)


def main():
    trials = []
    for name in FOLDERS:
        source = ROOT / name / 'result.json'
        raw = source.read_bytes()
        report = json.loads(raw)
        rows = report.get('samples', [])
        passed = report.get('status') == 'readonly_camera_timing_complete'
        timing = {
            'max_camera_poll_ms': max((s['camera_poll_ms'] for s in rows), default=None),
            'max_read_ms': max((s['read_ms'] for s in rows), default=None),
            'max_read_gap_ms': max((s.get('read_gap_ms') or 0 for s in rows), default=None),
        }
        observed_off = bool(rows) and all(
            len(s['torque']) == 16 and not any(s['torque'])
            and len(s['hardware_error']) == 16 and not any(s['hardware_error'])
            and len(s['watchdog']) == 16 and 255 not in s['watchdog']
            for s in rows
        )
        assert report['servo_write_commands_sent'] == 0
        if passed:
            assert report['timing_passed'] and observed_off
            assert all(value is not None and value <= 200 for value in timing.values())
            assert report['elapsed_seconds'] >= 12
            assert report['camera_recording']['status'] == 'complete'
        trials.append(dict(
            folder=name, result_sha256=hashlib.sha256(raw).hexdigest(),
            status=report['status'], error=report.get('error'), sample_count=len(rows),
            elapsed_seconds=report.get('elapsed_seconds'),
            all_samples_off_and_error_free=observed_off if rows else None,
            min_voltage_v=min((min(s['voltage_v']) for s in rows), default=None),
            camera_frames=report.get('camera_recording', {}).get('frame_count'),
            servo_write_commands_sent=0, **timing,
        ))
    assert [t['status'] for t in trials] == [
        'stopped', 'readonly_camera_timing_complete', 'readonly_camera_timing_complete'
    ]
    review = dict(
        trials=trials, completed_checks=2,
        repeated_readonly_check_needed=False,
        interpretation='The first trial failed at ID1 PING; its saved generic error does not establish why. The next two completed 12s with torque observed OFF and read gaps at most 63ms.',
        limits='This checks camera and read timing with torque OFF. Window dragging was not instrumented; loaded movement and physical cube rotation are not verified by these results.',
        next_required_observation='One existing grasp/contact-view run: show the index/middle fingertips against the cube after the grasp, then remove the cube and confirm OFF.',
        next_launcher='leap_cube_contact_view.cmd',
        motor_targets_or_limits_changed=False,
    )
    destination = ROOT / 'right_hand_rotation_20261023' / 'camera_readonly_hardware_review.json'
    destination.write_text(json.dumps(review, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(review, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
