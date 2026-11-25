"""Small, device-free inventory of recorded work-grip and next-turn sources."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
BASE = ROOT / 'right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110'


def main():
    actual = json.loads((ROOT / 'right_hand_rotation_20261112/stage2_actual_review/actual_work_grip_end_reference.json').read_bytes())
    end = np.asarray(actual['goal_ticks'])
    first = dict(np.load(ROOT / 'right_hand_rotation_20261104/forward_hold/candidate.npz', allow_pickle=False))
    yaw = first['goals'][np.asarray(first['phases']) == 'stage_7_horizontal_yaw']
    print(json.dumps({'first_yaw_begin': yaw[0].tolist(), 'first_yaw_end': yaw[-1].tolist(),
                      'work_to_yaw_delta': (yaw[0]-end).tolist(), 'work_to_yaw_minimum_steps': int(np.ceil(np.max(abs(yaw[0]-end))/2)),
                      'phase_counts': {str(p): int(np.count_nonzero(first['phases']==p)) for p in np.unique(first['phases'])}}))
    for name in ('initial_second_turn_hold_replay', 'initial_to_reconfigured_first90',
                 'initial_to_reached_workstation90', 'resumed_next90_joined_branch',
                 'opposing_workspace_next90', 'actual_A_next_two_contact_centered_teacher',
                 'robust_mu025_complete_cycle75ms_replay_v39'):
        folder = BASE / name
        path = folder / 'research_goals.npz'
        if not path.exists():
            print(json.dumps({'name': name, 'goals_exist': False})); continue
        z = dict(np.load(path, allow_pickle=False)); q = z['goals']
        delta = np.max(abs(q - end), axis=1); i = int(np.argmin(delta))
        print(json.dumps({'name': name, 'goals': len(q), 'start': z.get('start', []).tolist(),
                          'nearest_to_actual_goal': i, 'nearest_difference_ticks': float(delta[i]),
                          'first': q[0].tolist(), 'last': q[-1].tolist(),
                          'goal3404': q[3404].tolist() if len(q) > 3404 else None}))
        report_path = folder / 'review.json'
        if report_path.exists():
            r = json.loads(report_path.read_bytes())
            print(json.dumps({'review': name, 'scalars': {k: v for k, v in r.items() if not isinstance(v, (dict, list))},
                              'stage_names': [s.get('stage', s.get('name')) for s in r.get('stages', [])],
                              'segment_names': [s.get('stage', s.get('name')) for s in r.get('segments', [])]}))
    source = ROOT / 'right_hand_rotation_20261112/stage2_preparation/direct_work_grip_v1/review.json'
    r = json.loads(source.read_bytes())
    print(json.dumps({'stage2_comparisons': [{k: x.get(k) for k in ('target', 'order', 'moving_steps', 'stop',
           'supported_samples', 'hold_samples', 'upper_certified_samples', 'tilt_deg', 'orientation_error_deg')}
           for x in r['rows']]}))


if __name__ == '__main__':
    main()
