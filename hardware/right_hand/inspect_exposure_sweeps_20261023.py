"""Compact metadata for saved camera sweeps; no device access."""
from collections import Counter
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
for folder in sorted(ROOT.glob('camera_exposure_sweep_20261023_*'), reverse=True)[:6]:
    path = folder / 'result.json'
    if not path.is_file():
        continue
    result = json.loads(path.read_bytes())
    print(json.dumps(dict(folder=folder.name, status=result['status'],
        restoration=result.get('restoration'),
        profiles=[dict(name=p['name'], settings=p['settings'],
            frames=len(p.get('frames', [])),
            ids=dict(Counter(str(row['ids']) for row in p.get('frames', []))),
            usable_pair_frames=p.get('usable_pair_frames'), best_image=p.get('best_image'))
            for p in result.get('profiles', [])]), ensure_ascii=False, indent=2))
