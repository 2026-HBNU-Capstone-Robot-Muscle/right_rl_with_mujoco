"""Read source only. Find active conversion/import sites without loading controllers."""
from pathlib import Path
import json
r=Path(__file__).resolve().parent
for p in r.glob('*.cmd'):
    if p.name in ('leap_sac.cmd','leap_loaded_policy.cmd','leap_sac_start_capture.cmd'):
        print(p.name,p.read_text(encoding='utf-8-sig',errors='replace'))
for p in r.glob('*.py'):
    if p.name.startswith(('audit','test','inspect','sac_')):continue
    lines=p.read_text(encoding='utf-8-sig',errors='replace').splitlines()
    hits=[dict(line=i,text=t.strip()) for i,t in enumerate(lines,1) if ('from leap_hw import' in t or 'SIM_TO_ID=' in t or 'sim_to_hw(' in t)]
    if hits:print(json.dumps(dict(file=p.name,hits=hits),ensure_ascii=True))
