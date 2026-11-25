"""Rigid-cube orientation targets for the user's vertical/right/vertical sequence.

The six center-marker IDs follow cube_board.py: 0:+z, 1:-z, 2:+x,
3:-x, 4:+y, 5:-y. These are whole-cube rotations, not layer twists.
"""

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
NORMALS = np.asarray([[0, 0, 1], [0, 0, -1], [1, 0, 0],
                      [-1, 0, 0], [0, 1, 0], [0, -1, 0]], float)
COLORS = ['white', 'yellow', 'red', 'orange', 'blue', 'green']


def rotate_axis(axis, degrees):
    axis = np.asarray(axis, float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    theta = np.deg2rad(degrees)
    c, s = np.cos(theta), np.sin(theta)
    cross = np.asarray([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return c * np.eye(3) + (1 - c) * np.outer(axis, axis) + s * cross


def face_toward(R, direction):
    scores = (R @ NORMALS.T).T @ direction
    return int(np.argmax(scores))


def main():
    R = np.eye(3)
    initial_up = R @ NORMALS[5]
    initial_front = R @ NORMALS[3]
    phases = []

    def record(name):
        up_id = face_toward(R, initial_up)
        front_id = face_toward(R, initial_front)
        phases.append(dict(name=name, top_id=up_id, top_color=COLORS[up_id],
                           front_id=front_id, front_color=COLORS[front_id],
                           orientation_matrix=R.tolist()))

    record('start')
    right = np.cross(initial_up, initial_front)
    R = rotate_axis(right, 90) @ R
    record('vertical_flip_1')
    R = rotate_axis(initial_up, -90) @ R
    record('right_face_to_front')
    R = rotate_axis(right, 90) @ R
    record('vertical_flip_2')
    expected = [('green', 'orange'), ('red', 'green'),
                ('red', 'yellow'), ('white', 'red')]
    for row, pair in zip(phases, expected):
        if (row['top_color'], row['front_color']) != pair:
            raise AssertionError(f"{row['name']}: {row['top_color']}/{row['front_color']} != {pair}")
    report = dict(status='offline_ordered_whole_cube_goal_geometry',
                  motor_commands_sent=0, phases=phases,
                  caveat='This checks the discrete target orientations only. It does not model contact, finger motion, camera visibility, or physical success.')
    (ROOT / 'sac_ordered_cube_goal_result.json').write_text(
        json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'phases'} |
                     {'phases': [{k: v for k, v in row.items() if k != 'orientation_matrix'}
                                 for row in phases]}, indent=2))


if __name__ == '__main__':
    main()
