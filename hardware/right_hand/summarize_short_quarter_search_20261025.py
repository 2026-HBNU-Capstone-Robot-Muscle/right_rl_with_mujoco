import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_075405_loaded_model'
for name in ('short_turn_search','tip_clearance_search_v2','tip_clearance_search_v3'):
    p=OUT/name/'summary.json'
    if not p.exists():continue
    r=json.loads(p.read_bytes());good=[c for c in r['cases'] if c['complete_nominal_model_pass']]
    print(name,'total',len(r['cases']),'passing',len(good))
    for c in good:print(json.dumps(c))
    if name=='tip_clearance_search_v3':
        for c in r['cases']:
            if c['gain_scale']==.5 and c['stop_seconds']>60 and c['ring_tip_offset_ticks']==-56:
                print('LOW_GAIN_FIRST_CONTACT',{k:c.get(k) for k in ('thumb_tip_offset_ticks','ring_tip_offset_ticks','first_self_contact')})
    if not good:
        print('failure_examples',[{k:c.get(k) for k in ('gain_scale','stop_seconds','thumb_tip_offset_ticks','ring_tip_offset_ticks','first_self_contact')} for c in r['cases'] if c.get('gain_scale')==.5 and c['stop_seconds']>60])
v=json.loads((OUT/'short_quarter_native_verification.json').read_bytes())
for mode in ('moving_path','matched_stationary_control','empty_path'):
    rows=[r for r in v['cases'] if r['mode']==mode]
    print('VERIFIED',mode,'passing',sum(r['complete_model_pass'] for r in rows),'of',len(rows))
