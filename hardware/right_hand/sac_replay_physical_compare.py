"""Compare stage-2 to stage-4 cube motion in a real capture and offline replay."""

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent


def angle_deg(a, b):
    qa, qb = np.asarray(a, float), np.asarray(b, float)
    qa /= np.linalg.norm(qa)
    qb /= np.linalg.norm(qb)
    return float(2 * np.degrees(np.arccos(np.clip(abs(qa @ qb), 0, 1))))


def main():
    physical = json.loads((ROOT / 'sac_latest_orientation_result.json').read_text())
    relative = json.loads((ROOT / 'sac_relative_cube_motion_result.json').read_text())
    real = physical['two_center_by_phase']
    simulated_by_friction = {}
    for friction in (1, 8):
        simulation = json.loads((ROOT / f'sac_sequence_replay_friction_{friction}_result.json').read_text())
        sim = {f"stage_{r['stage']}": r for r in
               simulation['candidate_results'][0]['stage_endpoints'] if r['step'] == 70}
        simulated_by_friction[str(friction)] = dict(
            rotation_deg=angle_deg(sim['stage_2']['cube_quat_wxyz'],
                                   sim['stage_4']['cube_quat_wxyz']),
            center_displacement_mm=float(1000 * np.linalg.norm(
                np.asarray(sim['stage_4']['cube_center_m']) -
                np.asarray(sim['stage_2']['cube_center_m']))),
            face_changed=simulation['candidate_results'][0]['face_changed'],
            two_finger_steps=sum(v['two_finger_steps'] for v in
                                 simulation['candidate_results'][0]['contact_steps'].values()))
    report = dict(status='offline_sim_to_real_stage_motion_comparison', motor_commands_sent=0,
                  interval='stage_2_end_to_stage_4_end',
                  physical_camera_rotation_deg=real['stage_4']['rotation_from_stage_2_deg'],
                  physical_green_normal_change_deg=real['stage_4']['green_face_normal_change_deg'],
                  physical_center_displacement_mm=float(1000 * np.linalg.norm(
                      np.asarray(real['stage_4']['center_camera_m']) -
                      np.asarray(real['stage_2']['center_camera_m']))),
                  physical_mount_relative_rotation_deg=relative['stage_2_to_4']['mount_relative_cube_rotation_deg'],
                  physical_mount_relative_displacement_mm=relative['stage_2_to_4']['mount_relative_cube_translation_mm'],
                  simulated_by_friction=simulated_by_friction,
                  caveat='Physical mount frame and simulator world frame need not align; only rotation magnitude and displacement magnitude are compared. ID41 is assumed rigid to the palm, and single-marker pose has uncertainty. The sim start pose is provisional. Friction scale 8 multiplies all geom sliding friction in memory and is a sensitivity probe, not a fitted material value.')
    (ROOT / 'sac_replay_physical_compare_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
