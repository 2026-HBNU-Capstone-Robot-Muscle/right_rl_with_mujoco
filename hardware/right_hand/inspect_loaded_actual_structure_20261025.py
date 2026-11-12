import json
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parent;D=ROOT/'loaded_quarter_trial_20261025_171607_019752'
r=json.loads((D/'result.json').read_bytes());f=json.loads((D/'camera/frames.json').read_bytes())
print('RESULT_KEYS',list(r));print('SAMPLE_FIRST',r['samples'][0]);print('SAMPLE_TURN',next(s for s in r['samples'] if s['phase']=='stage_6_loaded_quarter_turn'))
print('FRAMES_TYPE',type(f).__name__)
if isinstance(f,dict):print('FRAMES_KEYS',list(f));f=next(v for v in f.values() if isinstance(v,list) and v and isinstance(v[0],dict))
print('FRAMES_COUNT',len(f));print('FRAMES_FIRST',f[:2]);print('FRAME_PHASES',dict(Counter(x.get('phase') for x in f)))
for p in ('sac_current_status.md','professor_progress_20261015.md','right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/loaded_quarter_trial_20261025/readiness_review.json'):
    t=(ROOT/p).read_text(encoding='utf-8');print('FILE',p);print(t[:3300])
