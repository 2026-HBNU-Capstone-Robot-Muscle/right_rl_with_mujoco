import json
from pathlib import Path
r=Path(__file__).resolve().parent
for name in ('sac_reachable_empty_path_id16capped.json','leap_id12_backoff_empty_result.json','sac_supported_reset_result.json'):
    d=json.loads((r/name).read_text(encoding='utf-8'))
    print(name, 'keys', list(d))
    if 'samples' in d:
        print('sample_count',len(d['samples']),'sample_keys',list(d['samples'][0]))
        for phase in dict.fromkeys(x['phase'] for x in d['samples']):
            row=next(x for x in d['samples'] if x['phase']==phase)
            print('phase',phase,'keys',list(row))
    if 'start_ticks' in d:print('start',d['start_ticks'],'goals',len(d['goals_id_order']))
for name in ('sac_current_status.md','leap_resume.ps1'):
    print(name,'\n',(r/name).read_text(encoding='utf-8-sig')[:2000])
