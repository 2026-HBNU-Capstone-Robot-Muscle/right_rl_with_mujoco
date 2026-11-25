"""Offline comparison of saved motor current before and after cube placement."""

import json
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
RUNS = ('leap_support_capture_result.json', 'leap_anchor_capture_result.json')


def robust_tail(rows, count=20):
    last = rows[-count:]
    return np.median(np.asarray([r['current_ma'] for r in last], dtype=float), axis=0)


def main():
    report = {'status': 'offline_motor_current_comparison', 'motor_commands_sent': 0,
              'runs': []}
    for filename in RUNS:
        data = json.loads((ROOT / filename).read_text(encoding='utf-8'))
        samples = data.get('samples', [])
        phases = Counter(row['phase'] for row in samples)
        stage = [row for row in samples if row['phase'] == 'stage_0']
        placed = [row for row in samples if row['phase'] == 'place_cube']
        row = dict(file=filename, status=data.get('status'), phase_counts=phases)
        if len(stage) >= 4 and len(placed) >= 20:
            before = robust_tail(stage, 4)
            after = robust_tail(placed)
            change = after - before
            row.update(stage0_tail_current_ma=before.tolist(),
                       placed_tail_current_ma=after.tolist(),
                       difference_ma=change.tolist(),
                       largest_absolute_changes=[dict(id=int(i + 1), delta_ma=float(change[i]))
                                                 for i in np.argsort(abs(change))[-6:][::-1]])
        report['runs'].append(row)
    report['caveat'] = ('Motor currents are influenced by pose tracking, friction, voltage and support; '
                        'a current difference does not uniquely locate cube contact.')
    (ROOT / 'sac_current_contact_probe_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps([dict(file=r['file'], phases=r['phase_counts'],
                           largest_changes=r.get('largest_absolute_changes'))
                      for r in report['runs']], indent=2))


if __name__ == '__main__':
    main()
