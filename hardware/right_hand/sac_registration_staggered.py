"""Estimate a provisional cube reset from ID40-before / cube-after frames.

No device access. A fixed camera alone does not prove that the hand mount
remained fixed during cube placement; the report retains that limitation.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from hand_registration_geometry import candidate

ROOT = Path(__file__).resolve().parent


def mean_rotation(matrices):
    avg = np.mean(np.asarray(matrices, dtype=float), axis=0)
    u, _, vt = np.linalg.svd(avg)
    correction = np.diag([1.0, 1.0, np.linalg.det(u @ vt)])
    return u @ correction @ vt


def spread_mm(values):
    arr = np.asarray(values, dtype=float)
    center = np.median(arr, axis=0)
    return (np.linalg.norm(arr - center, axis=1).max() * 1000.0)


def estimate(folder, hand_count=5, cube_count=30):
    session = json.loads((folder / 'result.json').read_text(encoding='utf-8'))
    rows = session['frames']
    hand = [r for r in rows if r['phase'] == 'stage_0' and r.get('pose')
            and 'R_hand' in r['pose'] and 40 in r.get('detected_ids', [])][-hand_count:]
    cube = [r for r in rows if r['phase'] == 'place_cube' and r.get('pose')
            and r['pose'].get('n_cube_markers', 0) >= 2
            and 'R_cube' in r['pose'] and 't_cube' in r['pose']][-cube_count:]
    if len(hand) < 3 or len(cube) < 5:
        raise RuntimeError(f'Insufficient sequential markers: hand={len(hand)} cube={len(cube)}')
    cube_ids = [sorted(i for i in r['detected_ids'] if i < 40) for r in cube]
    if len(set(tuple(ids) for ids in cube_ids)) > 1:
        raise RuntimeError('Last cube frames changed marker faces; select a stationary interval')
    hand_t = np.median([r['pose']['t_hand'] for r in hand], axis=0)
    cube_t = np.median([r['pose']['t_cube'] for r in cube], axis=0)
    hand_R = mean_rotation([r['pose']['R_hand'] for r in hand])
    cube_R = mean_rotation([r['pose']['R_cube'] for r in cube])
    registration = candidate()
    world_R_marker = np.asarray(registration['hand_to_world_candidate'])
    grasp_marker = np.asarray(registration['grasp_site_in_marker_m_candidate'])
    cube_marker = hand_R.T @ (cube_t - hand_t)
    cube_world = np.array([0.11, 0.0, 0.03]) + world_R_marker @ (cube_marker - grasp_marker)
    world_R_cube = world_R_marker @ hand_R.T @ cube_R
    # MuJoCo convention: [w, x, y, z].
    import sys
    sys.path.insert(0, str(ROOT.parent / 'work' / 'Leap_Test' / 'vision' / 'rubik_center'))
    import track_cube
    quat_world = np.asarray(track_cube.mat_to_quat(world_R_cube), dtype=float)
    report = {
        'status': 'provisional_sequential_registration',
        'hardware_authorized': False,
        'source_folder': str(folder),
        'hand_frames': len(hand), 'cube_frames': len(cube),
        'hand_last_unix_s': hand[-1]['captured_at_unix_s'],
        'cube_first_unix_s': cube[0]['captured_at_unix_s'],
        'gap_s': cube[0]['captured_at_unix_s'] - hand[-1]['captured_at_unix_s'],
        'hand_camera_spread_mm': spread_mm([r['pose']['t_hand'] for r in hand]),
        'cube_camera_spread_mm': spread_mm([r['pose']['t_cube'] for r in cube]),
        'cube_ids': cube_ids[-1],
        'cube_center_in_marker_m': cube_marker.tolist(),
        'cube_center_in_sim_world_candidate_m': cube_world.tolist(),
        'cube_quat_world_candidate': quat_world.tolist(),
        'limitation': 'Hand motion between the ID40 and cube frames is unobservable; validate contact and mount stability before training or motor use.',
    }
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', type=Path)
    args = parser.parse_args()
    if args.folder is None:
        run = json.loads((ROOT / 'leap_registration_capture_result.json').read_text(encoding='utf-8'))
        folder = Path(run['vision_recording']['folder'])
    else:
        folder = args.folder
    report = estimate(folder)
    out = ROOT / 'sac_registration_staggered_result.json'
    out.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
