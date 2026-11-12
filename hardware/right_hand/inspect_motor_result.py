"""Print bounded motor-run metadata without dumping large sample logs."""
import collections,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
p=(ROOT/sys.argv[1]).resolve();p.relative_to(ROOT);r=json.loads(p.read_bytes())
print('FIELDS',json.dumps({k:(len(v) if isinstance(v,(dict,list)) else v) for k,v in r.items()},ensure_ascii=False))
for key in ('stop','error','stop_reason','camera_recording','passive_observation_policy','read_recovery_confirmation_events','read_failures','final_off_reference_valid'):
    v=r.get(key)
    if v is not None:print(key,json.dumps(v[-8:] if isinstance(v,list) else v,ensure_ascii=False)[:16000])
s=r.get('samples',[])
if s:
    print('PHASES',json.dumps(dict(collections.Counter(x['phase'] for x in s)),ensure_ascii=False))
    print('LIMITS',json.dumps(dict(voltage_min=min(min(x['voltage_v']) for x in s),current_peak=max(max(abs(v) for v in x['current_ma']) for x in s),tracking_peak=max(max(x['tracking_rad']) for x in s),last_t=s[-1]['t'])))
    print('LAST_SAMPLE',json.dumps(s[-1],ensure_ascii=False))
print('FOLDER',str(p.parent))
