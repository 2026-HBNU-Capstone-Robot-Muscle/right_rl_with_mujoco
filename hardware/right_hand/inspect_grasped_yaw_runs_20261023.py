"""Compact review of saved runs. Never opens a camera or motor bus."""
from collections import Counter
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def inspect(folder):
    result = json.loads((folder / 'result.json').read_bytes())
    samples = result.get('samples', [])
    keep = ('timestamp', 'status', 'phase', 'error', 'failed_phase',
            'all_torque_off_verified', 'settings_restored', 'hardware_authorized',
            'cube_placement_confirmed', 'cube_removal_confirmed', 'cube_present',
            'rotation_command_path_planned', 'rotation_command_path_completed',
            'rotation_observation_photo', 'camera_recording',
            'current_start_screen_accepted', 'enabled_at')
    review = {key: result[key] for key in keep if key in result}
    review['source'] = folder.name
    review['read_failures'] = result.get('read_failures', [])
    review['sample_phases'] = dict(Counter(s['phase'] for s in samples))
    review['top_level_shapes'] = {
        key: (len(value) if isinstance(value, (dict, list)) else type(value).__name__)
        for key, value in result.items()
        if key not in keep and key not in ('samples', 'write_events', 'reads')}
    if samples:
        review['last_sample'] = samples[-1]
        review['minimum_voltage_v'] = min(min(s['voltage_v']) for s in samples)
        review['maximum_current_ma'] = max(max(abs(x) for x in s['current_ma']) for s in samples)
        review['maximum_tracking_rad'] = max(max(s['tracking_rad']) for s in samples)
    worker = result.get('camera_exposure_worker', {})
    review['camera_exposure'] = {key: worker.get(key) for key in
        ('status', 'original_settings', 'applied', 'restoration')}
    return review


def main():
    folders = sorted(ROOT.glob('grasped_yaw_trial_20261023_*'), reverse=True)
    for folder in folders:
        if not (folder / 'result.json').is_file():
            continue
        review = inspect(folder)
        if folder != folders[0]:
            review = {key: review.get(key) for key in
                ('source', 'status', 'phase', 'error', 'failed_phase', 'sample_phases',
                 'all_torque_off_verified', 'settings_restored')}
        print(json.dumps(review, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
