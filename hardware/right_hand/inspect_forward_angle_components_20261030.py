import json
from pathlib import Path
R=Path(__file__).resolve().parent/'adaptive_yaw_loaded_20261029_160451_416563/offline_review'
for name in ('grid_rotation_review.json','physical_review.json'):
 r=json.loads((R/name).read_bytes());print(name,list(r.keys()))
 print(json.dumps({k:v for k,v in r.items() if 'compar' in k or 'angle' in k or 'deg' in k or 'turn' in k or 'nominal' in k},ensure_ascii=False))
