import json
from pathlib import Path
p=Path(__file__).resolve().parent/'backoff_video_review_20261016'
r=json.loads((p/'middle_contact_audit.json').read_text())
print(json.dumps({k:v for k,v in r['model_collision_metadata'].items() if k not in ('excluded_body_pairs','middle_geometries')},indent=2))
for row in r['summary']:
    print(row['phase'],row['last_sample_nearest'])
print('incorrect_distance_pairs',len(r['distance_disagreements']))
