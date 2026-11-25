"""Read the saved latest rehearsal without opening any devices."""
import json
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parent
r=json.loads((ROOT/'leap_supported_yaw_empty_result.json').read_bytes())
keys=('timestamp','status','phase','error','failed_phase','plan_version',
      'all_torque_off_verified','settings_restored','contact_release_operator_confirmed',
      'contact_release_threshold_ticks','contact_release_actual_ticks','final_off_reference_valid',
      'empty_suffix_photo','cube_was_allowed','physical_rotation_verified')
print(json.dumps({k:r.get(k) for k in keys},ensure_ascii=False,indent=2))
print('CAMERA',json.dumps(r.get('camera_recording'),ensure_ascii=False))
rows=r.get('samples',[])
print('SAMPLE_KEYS',list(rows[0]) if rows else [])
print('PHASES',dict(Counter(s['phase'] for s in rows)))
for phase in dict.fromkeys(s['phase'] for s in rows):
    rr=[s for s in rows if s['phase']==phase]
    print('PHASE',phase,'count',len(rr),'first',json.dumps(rr[0])[:2400],'last',json.dumps(rr[-1])[:2400])
print('FINAL_ON',json.dumps(r.get('final_on_state'),ensure_ascii=False)[:5000])
print('OFF',json.dumps(r.get('final_off_snapshots',[])[-1:],ensure_ascii=False)[:5000])
