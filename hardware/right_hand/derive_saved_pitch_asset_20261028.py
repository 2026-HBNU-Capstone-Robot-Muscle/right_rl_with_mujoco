"""Extract the identical loaded targets of two saved successful flips.

Saved-data work only. This creates no operator command and opens no device.
"""
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from loaded_thumb_rotation_plan_20261026 import LoadedThumbRotationPlan, TURN_PHASE
from loaded_repeat_plan_20261026 import LoadedRepeatPlan

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261028/two_axis_connection/saved_pitch'
SOURCES = (
    ('loaded_thumb_rotation_20261026_135418_442474', LoadedThumbRotationPlan),
    ('loaded_repeat_20261026_170932_163649', LoadedRepeatPlan),
)

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def extract(folder, plan_type):
    raw_path = ROOT / folder / 'result.json'
    raw = json.loads(raw_path.read_bytes())
    evidence_path = raw_path.with_name('command_completion_review.json')
    evidence = json.loads(evidence_path.read_bytes())
    assert evidence['source_sha256'] == sha(raw_path)
    assert raw['status'] == plan_type.STATUS
    assert all(raw[k] is True for k in ('all_torque_off_verified', 'settings_restored',
                                      'cube_placement_confirmed', 'cube_removal_confirmed'))
    plan = plan_type(raw_path.with_name('fresh_off_request.json'))
    rows = plan.plan(np.asarray(raw['accepted_enable_goal']))
    writes = [e for e in raw['write_events'] if e['address'] == 116]
    assert len(writes) == len(rows) + 2
    assert all(e['comm'] == 0 for e in raw['write_events'])
    np.testing.assert_array_equal([e['requested'] for e in writes[2:]], [q for _, q in rows])
    assert all(s['goal_readback'] == s['goal'] for s in raw['samples'])
    first = next(i for i, (phase, _) in enumerate(rows) if phase == TURN_PHASE)
    start = rows[first - 1][1].copy()
    phases = np.array([p for p, _ in rows[first:]])
    goals = np.array([q for _, q in rows[first:]], dtype=np.int64)
    np.testing.assert_array_equal(start, raw['placement_goal_ticks'])
    assert np.max(np.abs(np.diff(np.vstack([start, goals]), axis=0))) <= 2
    assert raw['requested_sample_period_s'] == .075
    visual = raw_path.parent / 'visual_review/physical_flip_review.json'
    observation = json.loads(visual.read_bytes())
    assert observation['result_sha256'] == sha(raw_path)
    if plan_type is LoadedThumbRotationPlan:
        assert observation['status'] == 'physical_forward_quarter_turn_demonstrated_once'
        assert observation['visual_target_orientation_reached'] is True
    else:
        assert observation['status'] == 'physical_forward_flip_reproduced_once'
        assert observation['qualitative_target_reached'] is True
    return start, goals, phases, {
        'source': folder,
        'raw_result_sha256': sha(raw_path),
        'command_evidence_sha256': sha(evidence_path),
        'visual_evidence_sha256': sha(visual),
        'all_queued_goals_match_raw_writes': True,
        'loaded_goal_count': len(goals),
        'phase_counts': dict(Counter(phases)),
        'start_ticks': start.tolist(),
        'end_ticks': goals[-1].tolist(),
        'observed_start': observation['observed_start'],
        'observed_end': observation['observed_end'],
        'qualitative_target_reached': True,
        'exact_3d_angle_measured': False,
    }

def main():
    a, b = [extract(*source) for source in SOURCES]
    for i in range(3):
        np.testing.assert_array_equal(a[i], b[i])
    OUT.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUT / 'loaded_pitch_path.npz', start=a[0], goals=a[1], phases=a[2],
                        sample_period_s=.075, motor_ids_in_sim_order=SIM_TO_MOTOR_IDS)
    report = {
        'status': 'two_historical_pitch_successes_identical_loaded_targets_verified',
        'sources': [a[3], b[3]],
        'asset': (OUT / 'loaded_pitch_path.npz').relative_to(ROOT).as_posix(),
        'asset_sha256': sha(OUT / 'loaded_pitch_path.npz'),
        'sample_period_s': .075,
        'nominal_loaded_seconds': len(a[1]) * .075,
        'raw_evidence_preserved': True,
        'physical_successes': 2,
        'arbitrary_direction_control_verified': False,
        'pitch_yaw_connection_verified': False,
        'hardware_execution_allowed': False,
        'real_devices_opened': False,
        'motor_commands_sent': 0,
        'next_step': 'Review the loaded transition between the reached pitch ending and the yaw start; do not repeat the old flip merely for evidence.',
    }
    (OUT / 'source_review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False))

if __name__ == '__main__':
    main()
