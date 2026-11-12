import json,re,hashlib
from pathlib import Path
R=Path(__file__).resolve().parent;rows=[]
for p in sorted(R.glob('*.json')):
    if p.stat().st_size>8_000_000:continue
    if not (re.search(r'2026092[3-9]',p.name) or p.name=='hand_coordinate_audit_result.json'):continue
    text=p.read_text(encoding='utf-8-sig',errors='replace')
    try:d=json.loads(text)
    except ValueError:continue
    matches=[]
    for token in ('if_mcp','mf_mcp','sim2arr','SIM2ARR','joint_names','sim_joint','goal_sim','sim_target','sim_index','motor_axis_landmarks'):
        if token in text:
            i=text.index(token);matches.append(dict(token=token,context=text[max(0,i-70):i+240]))
    if matches:rows.append(dict(file=p.name,timestamp=d.get('timestamp'),matches=matches[:5],sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
(R/'full_mapping_review_20261018/early_records.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(rows[:12],ensure_ascii=True,indent=2))
