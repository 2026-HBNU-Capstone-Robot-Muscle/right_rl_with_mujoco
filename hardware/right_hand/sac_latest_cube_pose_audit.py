"""Summarize saved cube-pose observations in the two October 11 trials."""

import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FOLDERS = ['sequence_vision_20261011_051349_782086',
           'sequence_vision_20261011_051517_532925']


def main():
    for name in FOLDERS:
        data = json.loads((ROOT / name / 'result.json').read_text(encoding='utf-8'))
        print(name)
        for phase in ['place_cube', 'stage_1', 'stage_2', 'stage_3',
                      'stage_4', 'stage_5', 'stage_6', 'remove_cube']:
            rows = [r for r in data['frames'] if r['phase'] == phase]
            poses = [r for r in rows if r.get('pose') and r['pose'].get('t_cube') is not None]
            ids = Counter(tuple(i for i in r.get('detected_ids', []) if i < 6) for r in rows)
            sample = poses[-1] if poses else None
            print(json.dumps(dict(phase=phase, frames=len(rows), cube_poses=len(poses),
                                  cube_ids={str(k): v for k, v in ids.items()},
                                  last_pose_keys=list(sample['pose']) if sample else [],
                                  last_pose=sample['pose'] if sample else None),
                             ensure_ascii=False))


if __name__ == '__main__':
    main()
