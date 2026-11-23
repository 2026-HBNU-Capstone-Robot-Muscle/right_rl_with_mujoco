import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
for name in ['support_first_trial_20261113_040308_069460','support_first_trial_20261113_040515_333756']:
    folder=ROOT/name;r=json.loads((folder/'result.json').read_bytes())
    p=folder/'empty_recovery_screen/review.json'
    print('RUN',name,flush=True)
    print('WRITES',len(r.get('write_events',[])),'ALL_OFF',r.get('all_torque_off_verified'),flush=True)
    if p.exists():
        review=json.loads(p.read_bytes());static=review.get('static_cases',[]);dynamic=review.get('dynamic_cases',[])
        nominal=[row for row in static if not any(row.get('offset_ticks',[]))]
        print(json.dumps(dict(static_passed=sum(x['passed'] for x in static),static_total=len(static),dynamic_passed=sum(x['passed'] for x in dynamic),dynamic_total=len(dynamic),nominal=nominal[:1],dynamic=dynamic[:1]),ensure_ascii=False),flush=True)
    else:print('SCREEN_LOG', (folder/'current_start_screen.log').read_text(encoding='utf-8',errors='replace')[-3500:])
