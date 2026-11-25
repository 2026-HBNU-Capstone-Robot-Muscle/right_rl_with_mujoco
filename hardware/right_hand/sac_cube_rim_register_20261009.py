"""Propagate same-run hand-read cube rim pose through ID40/ID41 bridge."""

import json
import argparse
import sys
from pathlib import Path

import cv2 as cv
import numpy as np

from hand_registration_geometry import candidate
from sac_registration_from_anchor import detect, marker_pose


ROOT = Path(__file__).resolve().parent
FOLDER = ROOT / 'sequence_vision_20261009_221628_356082'
sys.path.insert(0, str(ROOT.parent / 'work/Leap_Test/vision/rubik_center'))
import track_cube


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', type=Path, default=FOLDER)
    parser.add_argument('--empty-image', default='anchor_empty_stage0.png')
    parser.add_argument('--rim-result', default='sac_cube_rim_20261009_result.json')
    parser.add_argument('--output-file', default='sac_cube_rim_register_20261009_result.json')
    args = parser.parse_args()
    folder = args.folder if args.folder.is_absolute() else ROOT / args.folder
    source = json.loads((ROOT / args.rim_result).read_text(encoding='utf-8'))
    placed = Path(source.get('source', source.get('image', '')))
    if not placed.is_absolute():
        placed = ROOT / placed
    if folder not in placed.parents:
        raise RuntimeError('Rim frame and empty frame must be from the same capture folder')
    with np.load(ROOT / 'camera1_720.npz') as f:
        K, dist = f['K'].copy(), f['dist'].copy()
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    _, ec, ei, _ = detect(folder / args.empty_image, dictionary)
    _, pc, pi, _ = detect(placed, dictionary)
    Rh, th, _ = marker_pose(ec, ei, 40, .0282, K, dist)
    Ra0, ta0, _ = marker_pose(ec, ei, 41, .030, K, dist)
    Ra1, ta1, _ = marker_pose(pc, pi, 41, .030, K, dist)
    Rcam = np.asarray(source['cube_rotation_camera'])
    tcam = np.asarray(source['cube_center_camera_m'])
    Rah = Ra0.T @ Rh
    tah = Ra0.T @ (th - ta0)
    cube_anchor = Ra1.T @ (tcam - ta1)
    cube_hand = Rah.T @ (cube_anchor - tah)
    Rch = Rah.T @ Ra1.T @ Rcam
    reg = candidate()
    Rwh = np.asarray(reg['hand_to_world_candidate'])
    grasp = np.asarray(reg['grasp_site_in_marker_m_candidate'])
    cube_world = np.array([.11, 0, .03]) + Rwh @ (cube_hand - grasp)
    report = {'status': 'offline_same_run_image_registration', 'hardware_authorized': False,
              'source': str(placed), 'empty_image': str(folder / args.empty_image),
              'cube_center_hand_marker_m': cube_hand.tolist(),
              'cube_center_world_m': cube_world.tolist(),
              'cube_quat_world_candidate': np.asarray(track_cube.mat_to_quat(Rwh @ Rch)).tolist(),
              'marker_reprojection_px': source['marker_reprojection_px'],
              'additional_reprojection_px': source.get('rim_reprojection_px',
                                                       source.get('sticker_reprojection_px')),
              'caveat': 'Image-derived cube pose and marker-to-model transform are provisional; diagnostic only.'}
    (ROOT / args.output_file).write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
