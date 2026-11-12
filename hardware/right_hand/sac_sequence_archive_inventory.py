"""Inventory the already archived one-pass motor traces for paired analysis."""

import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main():
    rows = []
    for path in sorted(ROOT.glob('leap_sequence_flip_20*.json')):
        data = json.loads(path.read_text(encoding='utf-8'))
        samples = data.get('samples', [])
        phases = Counter(s['phase'] for s in samples)
        row = dict(file=path.name, status=data.get('status'),
                   empty_check=data.get('empty_check'),
                   physical_flip_verified=data.get('physical_flip_verified'),
                   phase_counts=phases, sample_count=len(samples),
                   error=data.get('error'))
        rows.append(row)
    result = dict(count=len(rows), status_counts=Counter(r['status'] for r in rows),
                  full_runs=[r for r in rows if r['phase_counts'].get('stage_6', 0) > 0],
                  empty_full_runs=[r for r in rows if r['empty_check'] and r['phase_counts'].get('stage_6', 0) > 0])
    (ROOT / 'sac_sequence_archive_inventory_result.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(count=result['count'], status_counts=result['status_counts'],
                          full_runs=[dict(file=r['file'], status=r['status'],
                                          empty=r['empty_check'], samples=r['sample_count'])
                                     for r in result['full_runs']]), indent=2))


if __name__ == '__main__':
    main()
