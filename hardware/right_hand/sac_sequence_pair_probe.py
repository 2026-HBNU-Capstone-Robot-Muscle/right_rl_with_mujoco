"""Compare archived empty versus cube-loaded seven-pose motor traces."""

import json
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
EMPTY = ['leap_sequence_flip_20261006_003351_919305.json',
         'leap_sequence_flip_20261006_003605_661164.json']
LOADED = ['leap_sequence_flip_20261006_010338_480518.json',
          'leap_sequence_flip_20261006_010447_824916.json']


def stage_medians(names):
    observations = defaultdict(list)
    inventory = []
    for name in names:
        data = json.loads((ROOT / name).read_text(encoding='utf-8'))
        inventory.append(dict(file=name, status=data.get('status'),
                              source_sha256=data.get('source_sha256'),
                              sample_count=len(data.get('samples', []))))
        for row in data.get('samples', []):
            if row['phase'].startswith('stage_'):
                observations[row['phase']].append(row)
    medians = {}
    for phase, rows in sorted(observations.items()):
        current = np.asarray([r['current_ma'] for r in rows], float)
        tracking = np.asarray([r['tracking_rad'] for r in rows], float)
        medians[phase] = dict(samples=len(rows),
                              median_absolute_current_ma=np.median(abs(current), axis=0).tolist(),
                              median_tracking_rad=np.median(tracking, axis=0).tolist())
    return inventory, medians


def main():
    empty_runs, empty = stage_medians(EMPTY)
    loaded_runs, loaded = stage_medians(LOADED)
    phase_rows = []
    for phase in sorted(set(empty) & set(loaded)):
        before = np.asarray(empty[phase]['median_absolute_current_ma'])
        after = np.asarray(loaded[phase]['median_absolute_current_ma'])
        delta = after - before
        phase_rows.append(dict(phase=phase, empty_samples=empty[phase]['samples'],
                               loaded_samples=loaded[phase]['samples'],
                               largest_differences=[dict(id=int(i + 1), delta_ma=float(delta[i]))
                                                    for i in np.argsort(abs(delta))[-5:][::-1]]))
    report = dict(status='offline_archived_current_comparison', motor_commands_sent=0,
                  empty_runs=empty_runs, loaded_runs=loaded_runs, phases=phase_rows,
                  caveat='Run-to-run changes in motor posture, timing, voltage and mechanics confound currents. Current differences are not direct cube pose measurements.')
    (ROOT / 'sac_sequence_pair_probe_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
