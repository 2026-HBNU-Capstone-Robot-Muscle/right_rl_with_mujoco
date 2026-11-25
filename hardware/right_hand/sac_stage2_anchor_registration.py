"""Map the October 11 stage-2 cube pose into candidate MuJoCo coordinates.

The cube and ID41 were captured together at stage 2. At stages 4-6 both ID40
and ID41 were visible, allowing an *approximate* same-run rigid-marker bridge.
No camera or motor is opened.
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np

from hand_registration_geometry import candidate
from sac_registration_from_anchor import detect, marker_pose

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'vision' / 'rubik_center'))
import track_cube


def angle_deg(a, b):
    d = a.T @ b
    return float(np.degrees(np.arccos(np.clip((np.trace(d) - 1) / 2, -1, 1))))


def main():
    relative = json.loads((ROOT / 'sac_relative_cube_motion_result.json').read_text())
    folder = Path(relative['source_folder'])
    recording = json.loads((folder / 'result.json').read_text(encoding='utf-8'))
    with np.load(ROOT / 'camera1_720.npz') as cal:
        K, dist = cal['K'].copy(), cal['dist'].copy()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    bridges = {}
    for phase in ('stage_4', 'stage_5', 'stage_6'):
        path = Path(recording['phase_end_images'][phase]['image'])
        _, corners, ids, _ = detect(path, dictionary)
        if 40 not in ids or 41 not in ids:
            bridges[phase] = dict(status='marker_pair_missing', ids=ids)
            continue
        Rh, th, eh = marker_pose(corners, ids, 40, .0282, K, dist)
        Ra, ta, ea = marker_pose(corners, ids, 41, .030, K, dist)
        bridges[phase] = dict(status='paired', ids=ids,
                              reprojection_px=dict(hand=eh, anchor=ea),
                              hand_R_anchor=(Rh.T @ Ra).tolist(),
                              hand_t_anchor_m=(Rh.T @ (ta - th)).tolist())
    valid = {phase: row for phase, row in bridges.items() if row['status'] == 'paired'}
    if not valid:
        raise RuntimeError('No archived same-frame ID40/ID41 pair')
    base = valid['stage_4']
    R_ha, t_ha = np.asarray(base['hand_R_anchor']), np.asarray(base['hand_t_anchor_m'])
    variation = {}
    for phase, row in valid.items():
        variation[phase] = dict(
            translation_shift_mm=float(1000 * np.linalg.norm(
                np.asarray(row['hand_t_anchor_m']) - t_ha)),
            rotation_shift_deg=angle_deg(R_ha, np.asarray(row['hand_R_anchor'])))
    consistent_phases = [phase for phase, row in variation.items()
                         if row['translation_shift_mm'] <= 5 and
                         row['rotation_shift_deg'] <= 5]
    cube = relative['phases']['stage_2']
    R_ac, t_ac = np.asarray(cube['cube_R_anchor']), np.asarray(cube['cube_t_anchor_m'])
    R_hc, t_hc = R_ha @ R_ac, t_ha + R_ha @ t_ac
    registration = candidate()
    R_wh = np.asarray(registration['hand_to_world_candidate'])
    grasp_h = np.asarray(registration['grasp_site_in_marker_m_candidate'])
    world_center = np.asarray([.11, 0, .03]) + R_wh @ (t_hc - grasp_h)
    world_quat = np.asarray(track_cube.mat_to_quat(R_wh @ R_hc)).tolist()
    report = dict(status='same_session_stage2_candidate', motor_commands_sent=0,
                  source_folder=str(folder), anchor_hand_pairs=bridges,
                  anchor_pair_variation=variation,
                  anchor_pair_consistent_phases=consistent_phases,
                  cube_center_in_hand_marker_m=t_hc.tolist(),
                  cube_center_world_m=world_center.tolist(),
                  cube_quat_world_candidate=world_quat,
                  caveat='Camera-to-sim hand transform remains a candidate; stage-2 cube and stage-4 hand/anchor are bridged across time assuming rigid ID41. Stage 6 has an inconsistent marker-pair estimate and must not be used for calibration. Offline diagnostic only.')
    out = ROOT / 'sac_stage2_anchor_registration_result.json'
    out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'anchor_hand_pairs'}, indent=2))


if __name__ == '__main__':
    main()
