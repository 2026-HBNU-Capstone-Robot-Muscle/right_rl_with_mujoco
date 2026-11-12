"""Compare same-run camera and MuJoCo stage-2→4 cube translations."""

import json
from pathlib import Path

import numpy as np

from hand_registration_geometry import candidate

ROOT = Path(__file__).resolve().parent


def main():
    physical = json.loads((ROOT / 'sac_relative_cube_motion_result.json').read_text())
    bridge = json.loads((ROOT / 'sac_stage2_anchor_registration_result.json').read_text())
    sim_file = ROOT / 'sac_recorded_encoder_replay_stage2_offset_1.875_0_0_friction_2_result.json'
    sim = json.loads(sim_file.read_text())
    t2 = np.asarray(physical['phases']['stage_2']['cube_t_anchor_m'])
    t4 = np.asarray(physical['phases']['stage_4']['cube_t_anchor_m'])
    real_anchor = t4 - t2
    R_ha = np.asarray(bridge['anchor_hand_pairs']['stage_4']['hand_R_anchor'])
    R_wh = np.asarray(candidate()['hand_to_world_candidate'])
    real_world = R_wh @ R_ha @ real_anchor
    sim_world = (np.asarray(sim['snapshots']['stage_4']['cube_center_m']) -
                 np.asarray(sim['snapshots']['stage_2']['cube_center_m']))
    denom = np.linalg.norm(real_world) * np.linalg.norm(sim_world)
    report = dict(status='offline_stage2_to_4_translation_compare', motor_commands_sent=0,
                  physical_anchor_delta_mm=(1000 * real_anchor).tolist(),
                  physical_candidate_sim_frame_delta_mm=(1000 * real_world).tolist(),
                  simulated_delta_mm=(1000 * sim_world).tolist(),
                  vector_error_mm=float(1000 * np.linalg.norm(sim_world - real_world)),
                  direction_angle_deg=float(np.degrees(np.arccos(np.clip(
                      (real_world @ sim_world) / denom, -1, 1)))),
                  physical_length_mm=float(1000 * np.linalg.norm(real_world)),
                  simulated_length_mm=float(1000 * np.linalg.norm(sim_world)),
                  physical_source='sac_relative_cube_motion_result.json',
                  sim_source=sim_file.name,
                  caveat='Camera-to-sim hand transform is provisional. Relative displacement is insensitive to its translation but not its rotation. Candidate friction/offset were tuned on this run; this is a diagnostic fit, not independent validation.')
    (ROOT / 'sac_stage2_motion_compare_result.json').write_text(
        json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
