"""Inspect saved result schemas, not motor devices."""
import json
from pathlib import Path
root=Path(__file__).resolve().parent/'loaded_repeat_20261026_170932_163649'
r=json.loads((root/'result.json').read_bytes());print('RESULT KEYS',list(r));print('PLACEMENT',r.get('placement_position_ticks'))
f=json.loads((root/'camera/frames.json').read_bytes());print('FRAMES TYPE',type(f).__name__);print(list(f)[:6] if isinstance(f,dict) else f[:1])
print('FIRSTFRAME',f['frames'][0]);print('FIRSTSTAMP',f['timestamps'][0]);print('PLACEMENTPHOTO',r['placement_photo']);print('BEFORE',r.get('prepared_empty_photo'))
print('SAMPLE KEYS',list(r['samples'][0]));print('SAMPLE SCALARS',{k:v for k,v in r['samples'][0].items() if not isinstance(v,(list,dict))})
