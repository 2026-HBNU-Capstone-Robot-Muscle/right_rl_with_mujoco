import re,json
from pathlib import Path
R=Path(__file__).resolve().parent;P=R/'right_hand_rotation_20261030/four_forward_review/index.html';s=P.read_text(encoding='utf-8');links=re.findall(r'(?:src|href)="([^"]+)"',s);missing=[]
for link in links:
 if link.startswith(('http:','https:','#','data:')):continue
 if not (P.parent/link).resolve().exists():missing.append(link)
assert not missing,missing
for rel in ('right_hand_rotation_20261030/single_forward/handoff.json','right_hand_rotation_20261028/two_axis_connection/current_progress.json'):
 r=json.loads((R/rel).read_bytes());assert r.get('next_command') is None and r.get('user_command') is None and not r.get('ready')
print('Review links exist:',len(links),'; new motion pointers remain disabled.')
