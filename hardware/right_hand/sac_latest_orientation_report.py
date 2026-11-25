"""Summarize two-center cube rotation during the latest real sequence capture."""

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
FOLDER = ROOT / 'sequence_vision_20261011_051517_532925'


def angle_deg(a, b):
    delta = np.asarray(a).T @ np.asarray(b)
    return float(np.degrees(np.arccos(np.clip((np.trace(delta) - 1) / 2, -1, 1))))


def green_normal_change_deg(a, b):
    green = np.array([0, -1.0, 0])
    na, nb = np.asarray(a) @ green, np.asarray(b) @ green
    return float(np.degrees(np.arccos(np.clip(na @ nb, -1, 1))))


def main():
    data = json.loads((FOLDER / 'result.json').read_text(encoding='utf-8'))
    selected = {}
    for phase in ('place_cube', 'stage_1', 'stage_2', 'stage_3',
                  'stage_4', 'stage_5', 'stage_6'):
        rows = [r for r in data['frames'] if r['phase'] == phase and
                r.get('pose') and r['pose'].get('n_cube_markers', 0) >= 2 and
                r['pose'].get('reproj', 100) <= 3]
        if rows:
            row = rows[-1]
            selected[phase] = dict(motor_t_s=row.get('motor_t_s'),
                                   ids=[i for i in row.get('detected_ids', []) if i < 6],
                                   reprojection_px=row['pose']['reproj'],
                                   R_cube=row['pose']['R_cube'],
                                   center_camera_m=row['pose']['t_cube'])
    reference = selected.get('stage_2')
    reference_rotation = None if reference is None else reference['R_cube']
    for phase, row in selected.items():
        row['rotation_from_stage_2_deg'] = (
            angle_deg(reference_rotation, row['R_cube']) if reference_rotation is not None else None)
        row['green_face_normal_change_deg'] = (
            green_normal_change_deg(reference_rotation, row['R_cube'])
            if reference_rotation is not None else None)
        row.pop('R_cube')
    face_up = {}
    for phase in ('stage_4', 'stage_5', 'stage_6'):
        rows = [r for r in data['frames'] if r['phase'] == phase and
                r.get('pose') and r['pose'].get('face_up') is not None and
                r['pose'].get('n_cube_markers', 0) >= 2]
        face_up[phase] = sorted({r['pose']['face_up'] for r in rows})
    report = dict(status='archived_two_center_orientation', motor_commands_sent=0,
                  source=str(FOLDER / 'result.json'), two_center_by_phase=selected,
                  observed_face_up_ids=face_up,
                  caveat='Single archived run. Two-center poses are camera-frame estimates; one-center/occluded intervals and human contact are not reconstructed.')
    (ROOT / 'sac_latest_orientation_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
