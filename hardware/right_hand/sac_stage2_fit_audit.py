"""Rank archived stage-2 replay sensitivity runs against independent observables."""

import json
from pathlib import Path

import numpy as np

from hand_registration_geometry import candidate

ROOT = Path(__file__).resolve().parent


def main():
    physical = json.loads((ROOT / 'sac_relative_cube_motion_result.json').read_text())
    bridge = json.loads((ROOT / 'sac_stage2_anchor_registration_result.json').read_text())
    target = physical['stage_2_to_4']
    R_ha = np.asarray(bridge['anchor_hand_pairs']['stage_4']['hand_R_anchor'])
    R_wh = np.asarray(candidate()['hand_to_world_candidate'])
    t2 = np.asarray(physical['phases']['stage_2']['cube_t_anchor_m'])
    t4 = np.asarray(physical['phases']['stage_4']['cube_t_anchor_m'])
    physical_world_delta_mm = 1000 * R_wh @ R_ha @ (t4 - t2)
    rows = []
    for path in sorted(ROOT.glob('sac_recorded_encoder_replay_stage2*_result.json')):
        run = json.loads(path.read_text(encoding='utf-8'))
        measure = run['stage_2_to_4']
        snapshots = run['snapshots']
        sim_delta_mm = 1000 * (
            np.asarray(snapshots['stage_4']['cube_center_m']) -
            np.asarray(snapshots['stage_2']['cube_center_m']))
        orientation_error_deg = float(np.linalg.norm([
            measure['green_tip_toward_initial_orange_front_deg'] -
            target['green_tip_toward_initial_orange_front_deg'],
            measure['orange_front_turn_about_initial_green_axis_deg'] -
            target['orange_front_turn_about_initial_green_axis_deg'],
        ]))
        rows.append(dict(source=path.name, offset_mm=run['selected_offset_mm'],
                         friction_scale=run.get('friction_scale', 1.0),
                         orientation_component_error_deg=orientation_error_deg,
                         translation_vector_error_mm=float(np.linalg.norm(
                             sim_delta_mm - physical_world_delta_mm)),
                         simulated_translation_mm=float(np.linalg.norm(sim_delta_mm)),
                         stage3_two_finger_steps=run['contact_steps']['stage_3']['two_finger_steps'],
                         stage4_two_finger_steps=run['contact_steps']['stage_4']['two_finger_steps'],
                         initial_penetration_mm=run['fixed_joint_hold_diagnostic']['initial_max_penetration_mm'],
                         initial_hold_drift_mm=run['fixed_joint_hold_diagnostic']['cube_center_drift_mm']))
    by_orientation = sorted(rows, key=lambda r: r['orientation_component_error_deg'])
    by_translation = sorted(rows, key=lambda r: r['translation_vector_error_mm'])
    report = dict(status='offline_same_run_model_fit_audit', motor_commands_sent=0,
                  candidate_count=len(rows),
                  physical_translation_length_mm=float(np.linalg.norm(physical_world_delta_mm)),
                  best_orientation_candidates=by_orientation[:5],
                  best_translation_candidates=by_translation[:5],
                  all_candidates=rows,
                  caveat='These offsets and friction factors were explored on the same physical trial; no held-out validation, external wood geometry, or calibrated camera-to-sim transform. Matching one component is not proof of transfer.')
    (ROOT / 'sac_stage2_fit_audit_result.json').write_text(
        json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'all_candidates'}, indent=2))


if __name__ == '__main__':
    main()
