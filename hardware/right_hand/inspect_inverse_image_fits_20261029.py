"""Summarize saved fit failures without printing large frame reports."""
import collections,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
O=ROOT/'gapped_inverse_trial_20261029_184252_272166/offline_review'
for name,key in (('grid_rotation_review.json','fits'),('marker_labelled_grid_trace.json','rows')):
    p=O/name
    if not p.exists():continue
    rows=json.loads(p.read_bytes())[key]
    groups=collections.defaultdict(list)
    for row in rows:groups[row['phase']].append(row)
    for phase,items in groups.items():
        print(json.dumps(dict(source=name,phase=phase,frames=len(items),
            ids=collections.Counter(','.join(map(str,sorted(i.get('ids',[])))) for i in items),
            sticker_counts=collections.Counter(str(i.get('sticker_count')) for i in items),
            accepted=sum(i.get('accepted',i.get('rmse_px',100)<2) for i in items),
            example=next((i for i in items if 5 in i.get('ids',[])),items[0])),ensure_ascii=False))
