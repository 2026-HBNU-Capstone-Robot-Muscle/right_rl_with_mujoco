"""Compare archived cube motion relative to the rigid ID41 mount, offline."""

import json
from pathlib import Path

import cv2
import numpy as np

from sac_registration_from_anchor import detect, marker_pose


ROOT = Path(__file__).resolve().parent
FOLDER = ROOT / 'sequence_vision_20261011_051517_532925'


def rotation_angle_deg(a, b):
    delta = np.asarray(a).T @ np.asarray(b)
    return float(np.degrees(np.arccos(np.clip((np.trace(delta) - 1) / 2, -1, 1))))


def main():
    vision = json.loads((FOLDER / 'result.json').read_text(encoding='utf-8'))
    with np.load(ROOT / 'camera1_720.npz') as calibration:
        intrinsic, distortion = calibration['K'].copy(), calibration['dist'].copy()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    phases = {}
    for phase in ('stage_2', 'stage_3', 'stage_4', 'stage_5', 'stage_6'):
        end = vision['phase_end_images'][phase]
        frame_path = Path(end['image'])
        _, corners, ids, _ = detect(frame_path, dictionary)
        if 41 not in ids:
            phases[phase] = dict(status='ID41_not_detected', detected_ids=ids)
            continue
        rows = [r for r in vision['frames'] if r['phase'] == phase and
                r['captured_at_unix_s'] == end['captured_at_unix_s']]
        if len(rows) != 1 or rows[0].get('pose') is None:
            phases[phase] = dict(status='end_pose_unavailable', detected_ids=ids)
            continue
        row = rows[0]
        cube = row['pose']
        if cube.get('n_cube_markers', 0) < 2 or cube.get('reproj', 100) > 3:
            phases[phase] = dict(status='two_center_cube_pose_unavailable', detected_ids=ids)
            continue
        R_a, t_a, reproj = marker_pose(corners, ids, 41, .030, intrinsic, distortion)
        R_c, t_c = np.asarray(cube['R_cube']), np.asarray(cube['t_cube'])
        phases[phase] = dict(status='paired_same_frame', detected_ids=ids,
                             camera_anchor_reprojection_px=reproj,
                             camera_cube_reprojection_px=cube['reproj'],
                             anchor_R_camera=R_a.tolist(), anchor_t_camera_m=t_a.tolist(),
                             cube_R_camera=R_c.tolist(), cube_t_camera_m=t_c.tolist(),
                             cube_R_anchor=(R_a.T @ R_c).tolist(),
                             cube_t_anchor_m=(R_a.T @ (t_c - t_a)).tolist())
    a, b = phases['stage_2'], phases['stage_4']
    report = dict(status='offline_cube_motion_relative_to_ID41', motor_commands_sent=0,
                  source_folder=str(FOLDER), phases=phases,
                  user_observation={
                      'intended_first_action': 'forward whole-cube flip',
                      'observed_front_face': 'orange changed to yellow as cube turned left',
                      'observed_top_face': 'green remained on top throughout',
                      'human_contact': 'held robot mount only; did not touch cube',
                  })
    if a['status'] == b['status'] == 'paired_same_frame':
        R2, R4 = np.asarray(a['cube_R_anchor']), np.asarray(b['cube_R_anchor'])
        green = np.asarray([0.0, -1.0, 0.0])
        green2, green4 = R2 @ green, R4 @ green
        # ID3 is the orange front center in the user's placement image.
        orange_front2 = R2 @ np.asarray([-1.0, 0.0, 0.0])
        orange_front4 = R4 @ np.asarray([-1.0, 0.0, 0.0])
        forward_tip_deg = float(np.degrees(np.arctan2(
            green4 @ orange_front2, green4 @ green2)))
        yaw_deg = float(np.degrees(np.arctan2(
            np.cross(orange_front2, orange_front4) @ green2,
            orange_front2 @ orange_front4)))
        rvec, _ = cv2.Rodrigues(R4 @ R2.T)
        axis = rvec.reshape(3) / np.linalg.norm(rvec)
        report['stage_2_to_4'] = dict(
            camera_cube_rotation_deg=rotation_angle_deg(a['cube_R_camera'], b['cube_R_camera']),
            camera_mount_rotation_deg=rotation_angle_deg(a['anchor_R_camera'], b['anchor_R_camera']),
            mount_relative_cube_rotation_deg=rotation_angle_deg(a['cube_R_anchor'], b['cube_R_anchor']),
            camera_cube_translation_mm=float(1000 * np.linalg.norm(
                np.asarray(b['cube_t_camera_m']) - np.asarray(a['cube_t_camera_m']))),
            camera_mount_translation_mm=float(1000 * np.linalg.norm(
                np.asarray(b['anchor_t_camera_m']) - np.asarray(a['anchor_t_camera_m']))),
            mount_relative_cube_translation_mm=float(1000 * np.linalg.norm(
                np.asarray(b['cube_t_anchor_m']) - np.asarray(a['cube_t_anchor_m']))),
            mount_relative_green_normal_change_deg=float(np.degrees(np.arccos(
                np.clip(green2 @ green4, -1, 1)))),
            green_tip_toward_initial_orange_front_deg=forward_tip_deg,
            orange_front_turn_about_initial_green_axis_deg=yaw_deg,
            rotation_axis_anchor=axis.tolist(),
            rotation_axis_to_initial_green_normal_deg=float(np.degrees(np.arccos(
                np.clip(abs(axis @ green2), 0, 1)))))
    report['caveat'] = ('ID41 is assumed rigid to the palm. Planar single-marker pose can be noisy; '
                        'same-frame two-center cube poses reduce timing ambiguity but are not contact measurements.')
    (ROOT / 'sac_relative_cube_motion_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    concise = {k: v for k, v in report.items() if k != 'phases'}
    concise['phase_status'] = {k: v['status'] for k, v in phases.items()}
    print(json.dumps(concise, indent=2))


if __name__ == '__main__':
    main()
