"""Measure observed 0/1/2-center gaps in already-recorded camera runs."""

import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RUNS = [
    ('old_flip_sweep', ROOT / 'vision_sweep_20261006_011011_994373' / 'result.json', None),
    ('latest_stage0_placement', ROOT / 'sequence_vision_20261010_045958_610104' / 'result.json', 'place_cube'),
]


def summarize(rows, time_field):
    rows = [r for r in rows if time_field in r and r[time_field] is not None]
    rows.sort(key=lambda row: row[time_field])
    counts = Counter(min(2, int(row.get('n_cube_markers', 0))) for row in rows)
    gaps = []
    last_two = None
    for row in rows:
        t = float(row[time_field])
        if row.get('n_cube_markers', 0) >= 2:
            if last_two is not None:
                gaps.append(t - last_two)
            last_two = t
    two_times = [float(r[time_field]) for r in rows if r.get('n_cube_markers', 0) >= 2]
    return dict(frames=len(rows), center_count_frames={str(k): counts[k] for k in range(3)},
                two_center_percent=100 * counts[2] / len(rows) if rows else 0,
                first_two_center_s=two_times[0] if two_times else None,
                last_two_center_s=two_times[-1] if two_times else None,
                longest_between_two_center_s=max(gaps) if gaps else None,
                full_span_s=(float(rows[-1][time_field]) - float(rows[0][time_field])) if rows else 0)


def main():
    result = {}
    for label, path, phase in RUNS:
        data = json.loads(path.read_text(encoding='utf-8'))
        rows = data['frames']
        if phase:
            rows = [row for row in rows if row.get('phase') == phase]
        time_field = 'captured_at_unix_s' if phase else 'time_s'
        result[label] = summarize(rows, time_field)
    report = dict(status='archived_visibility_only', motor_commands_sent=0, runs=result,
                  caveat='Frame summary has no raw continuous images or ground-truth cube pose.')
    (ROOT / 'sac_center_visibility_archive_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
