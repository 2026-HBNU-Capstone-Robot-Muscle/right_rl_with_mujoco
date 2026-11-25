"""Compare two photographed stage-0 cube starts in the hand marker frame."""

import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent


def main():
    files = ['sac_cube_rim_register_20261009_result.json',
             'sac_cube_rim_register_2316_result.json']
    rows = [json.loads((ROOT / name).read_text(encoding='utf-8')) for name in files]
    centers = [np.asarray(row['cube_center_hand_marker_m']) for row in rows]
    delta = (centers[1] - centers[0]) * 1000
    report = {'status': 'offline_two_stage0_start_comparison', 'hardware_authorized': False,
              'sources': files, 'centers_hand_marker_m': [c.tolist() for c in centers],
              'delta_second_minus_first_mm': delta.tolist(),
              'center_distance_mm': float(np.linalg.norm(delta)),
              'midpoint_hand_marker_m': np.mean(centers, axis=0).tolist(),
              'caveat': 'Both poses use hand-read cube rim corners; their common systematic error is not removed.'}
    (ROOT / 'sac_two_start_consistency_20261009_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
