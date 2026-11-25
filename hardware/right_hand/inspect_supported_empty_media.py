import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
f=ROOT/'supported_yaw_empty_20261023_030552_675209'
r=json.loads((f/'camera/frames.json').read_bytes())
print('FRAMES',type(r['frames']).__name__,len(r['frames']))
print('FIRST',json.dumps(r['frames'][0])[:900])
print('LAST',json.dumps(r['frames'][-1])[:900])
print('TIMESTAMPS',str(r['timestamps'])[:600])
rr=json.loads((f/'result.json').read_bytes())
print('OFF_DRIFT',rr.get('final_off_endpoint_drift_ticks'))
print('ROOT_ERROR',rr.get('off_reference_capture_error'))
for s in rr.get('final_off_snapshots',[]):print('OFF_POSITION',s['position'])
