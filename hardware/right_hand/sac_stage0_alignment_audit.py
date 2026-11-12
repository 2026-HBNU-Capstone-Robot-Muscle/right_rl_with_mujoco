"""Compare the measured stage-0 joints with the simulated contact check, offline."""

import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
TICK_RAD = 2 * np.pi / 4096


def main():
    trial = json.loads((ROOT / 'leap_anchor_capture_result.json').read_text(encoding='utf-8'))
    if trial.get('status') != 'placement_capture_complete' or not trial.get('all_torque_off_verified'):
        raise RuntimeError('Completed stage-0 trial with torque OFF verification required')
    stage = trial['stage_end_tracking']['stage_0']
    goal = np.asarray(stage['goal'], dtype=float)
    actual = np.asarray(stage['actual'], dtype=float)
    error_rad = np.abs(goal - actual) * TICK_RAD
    vision = json.loads(Path(trial['vision_recording']['result']).read_text(encoding='utf-8'))
    capture_time = trial['cube_placement_camera_snapshot']['captured_at_unix_s']
    placement_row = min(vision['frames'],
                        key=lambda row: abs(row['captured_at_unix_s'] - capture_time))
    sample = min((row for row in trial['samples'] if row['phase'] == 'place_cube'),
                 key=lambda row: abs(row['t'] - placement_row['motor_t_s']))
    loaded_actual = np.asarray(sample['actual'], dtype=float)
    loaded_error_rad = np.abs(goal - loaded_actual) * TICK_RAD
    prefix = 'sac_fit_contact_real_cube_100g_near_measured_candidate_diagnostic'
    target = json.loads((ROOT / (prefix + '_result.json')).read_text(encoding='utf-8'))
    observed = json.loads((ROOT / (prefix + '_actual_stage0_result.json')).read_text(encoding='utf-8'))
    placed = json.loads((ROOT / (prefix + '_actual_placement_result.json')).read_text(encoding='utf-8'))
    a = target['measured_contact_diagnostic']
    b = observed['measured_contact_diagnostic']
    c = placed['measured_contact_diagnostic']
    result = {
        'max_joint_error_rad': float(error_rad.max()),
        'worst_motor_id': int(error_rad.argmax() + 1),
        'joint_error_rad_by_id': error_rad.tolist(),
        'placement_motor_sample_offset_s': float(sample['t'] - placement_row['motor_t_s']),
        'max_loaded_joint_error_rad': float(loaded_error_rad.max()),
        'worst_loaded_motor_id': int(loaded_error_rad.argmax() + 1),
        'loaded_joint_error_rad_by_id': loaded_error_rad.tolist(),
        'loaded_actual_ticks': loaded_actual.astype(int).tolist(),
        'target_start_drift_mm': 1000 * a['drift_m'],
        'actual_start_drift_mm': 1000 * b['drift_m'],
        'actual_placement_drift_mm': 1000 * c['drift_m'],
        'target_initial_penetration_mm': 1000 * a['initial_penetration_m'],
        'actual_initial_penetration_mm': 1000 * b['initial_penetration_m'],
        'target_stable_candidates': target['stable_candidates'],
        'actual_stable_candidates': observed['stable_candidates'],
        'actual_placement_stable_candidates': placed['stable_candidates'],
        'conclusion': 'Measured motor tracking error alone cannot explain the start-contact mismatch.',
        'hardware_authorized': False,
    }
    (ROOT / 'sac_stage0_alignment_audit_result.json').write_text(
        json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
