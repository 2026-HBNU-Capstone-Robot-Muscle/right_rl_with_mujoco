"""Inspect already-saved configuration evidence; no serial access."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parent
rows=[]
for name in ('hand_readonly_check_result.json','leap_drive_readonly_result.json','hand_base_settings_readonly_result.json','hand_tip_settings_readonly_result.json','hand12_readonly_diag_result.json','leap_id12_backoff_empty_result.json'):
    p=ROOT/name
    if not p.exists():continue
    d=json.loads(p.read_text(encoding='utf-8'));found=[]
    def walk(obj,trail=''):
        if isinstance(obj,dict):
            for k,v in obj.items():
                if any(t in k for t in ('homing','drive_mode','firmware')):found.append(dict(path=trail+'/'+k,value=v))
                elif isinstance(v,(dict,list)):walk(v,trail+'/'+k)
        elif isinstance(obj,list):
            for i,v in enumerate(obj):walk(v,trail+'/'+str(i))
    walk(d)
    row=dict(file=name,timestamp=d.get('timestamp'),evidence=found);rows.append(row)
    print(json.dumps(row))
(ROOT/'backoff_video_review_20261016'/'saved_configuration_audit.json').write_text(json.dumps(rows,indent=2))
