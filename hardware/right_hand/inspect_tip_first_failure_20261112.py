import json
from pathlib import Path
root=Path(__file__).resolve().parent
for folder in sorted(root.glob('tip_first_trial_20261112_17*'),reverse=True)[:4]:
    result=folder/'result.json'
    if not result.exists():continue
    data=json.loads(result.read_bytes())
    print('\nRUN',folder.name)
    for key in ['write_events','preflight_state','off_start_observations','last_issued_command','planned_final_command','last_issued_phase','error','all_torque_off_verified','settings_restored']:
        if key in data:print(key,json.dumps(data[key],ensure_ascii=False)[:1700])
    p=folder/'empty_recovery_screen/review.json'
    if p.exists():
        review=json.loads(p.read_bytes())
        print('REVIEW_KEYS',list(review))
        for key,value in review.items():
            if isinstance(value,list):print(key,'count',len(value),'first',json.dumps(value[:1],ensure_ascii=False)[:3500])
            elif isinstance(value,dict):print(key,json.dumps(value,ensure_ascii=False)[:1500])
            else:print(key,str(value)[:400])
    p=folder/'fresh_off_request.json'
    if p.exists():
        request=json.loads(p.read_bytes())
        print('REQUEST',json.dumps(request,ensure_ascii=False)[:4500])
