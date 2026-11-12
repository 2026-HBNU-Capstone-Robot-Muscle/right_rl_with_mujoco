"""Inspect saved operator runs only; no device APIs."""
import hashlib
import json
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261112/nominal_cycle_actual_review'; OUT.mkdir(exist_ok=True)
rows=[]
for folder in sorted(ROOT.glob('nominal_two_turn_trial_20261112_*')):
    path=folder/'result.json'
    if not path.exists():
        print(json.dumps({'folder':folder.name,'result_exists':False})); continue
    r=json.loads(path.read_bytes()); ss=r.get('samples',[])
    row={'folder':folder.name,'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
         **{k:r.get(k) for k in ('status','error','failed_phase','primary_motion_error','all_torque_off_verified','settings_restored','cube_placement_confirmed','second_rotation_attempted','cube_support_confirmed_before_off')},
         'samples':len(ss),'read_failures':r.get('read_failures',[]),
         'phase_counts':dict(Counter(s['phase'] for s in ss)),
         'photos':[p.name for p in folder.glob('*.jpg')],
         'top_level_error_fields':{k:v for k,v in r.items() if ('error' in k or 'fail' in k) and k not in ('samples','read_failures') and not isinstance(v,(list,dict))}}
    if ss:
        worst=max(ss,key=lambda s:max(s['tracking_rad']))
        row.update(min_voltage_v=min(min(s['voltage_v']) for s in ss),max_current_ma=max(max(abs(v) for v in s['current_ma']) for s in ss),
                   max_tracking_rad=max(worst['tracking_rad']),max_temperature_c=max(max(s['temperature_c']) for s in ss),
                   last_t=ss[-1]['t'],last_phase=ss[-1]['phase'],last_actual=ss[-1]['actual'],last_goal=ss[-1]['goal'],
                   worst_tracking_motor_id=worst['tracking_rad'].index(max(worst['tracking_rad']))+1,worst_tracking_phase=worst['phase'],
                   last_samples=[{k:s[k] for k in ('t','phase','actual','goal','tracking_rad')} for s in ss[-3:]])
    rows.append(row)
    (OUT/(folder.name+'.json')).write_text(json.dumps(row,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    brief={k:v for k,v in row.items() if k not in ('last_actual','last_goal','last_samples','source_sha256','read_failures','top_level_error_fields')}
    brief['read_failures']=len(row['read_failures']); print(json.dumps(brief,ensure_ascii=False))
(OUT/'inventory.json').write_text(json.dumps({'runs':rows,'review_device_accesses':0},indent=2,ensure_ascii=False)+'\n',encoding='utf-8')

if '--diagnose' in __import__('sys').argv:
    for row in rows:
        folder=ROOT/row['folder']; raw=json.loads((folder/'result.json').read_bytes())
        if row['samples']:
            samples=raw['samples']; last=samples[-1]['t']; tail=[s for s in samples if last-6<=s['t']<=last]
            small=[{'t':s['t'],'ID16_actual':s['actual'][15],'ID16_goal':s['goal'][15],
                    'ID16_current_ma':s['current_ma'][15]} for s in tail[::10]]
            print('ID16_TAIL',json.dumps(small))
            print('WRITE_COUNTS',row['folder'],len(raw.get('write_events',[])))
            print('TORQUE_EVENTS',json.dumps([e for e in raw.get('write_events',[]) if e.get('address')==64]))
        else:
            print('NO_MOTION',row['folder'],'writes',len(raw.get('write_events',[])))
        fp=folder/'camera/frames.json'
        if fp.exists():
            frames=json.loads(fp.read_bytes())['frames']
            if frames:
                points=[-1] if not row['samples'] else [int(len(frames)*.5),max(0,len(frames)-80),max(0,len(frames)-15),len(frames)-1]
                print('FRAMES',json.dumps([(folder/'camera'/frames[i]['image']).relative_to(ROOT).as_posix() for i in points]))
                if row['samples']:
                    end=raw['samples'][-1]['t']; enabled=raw['enabled_at']
                    timed=[]
                    for desired in (end-5,end-1,end-.1,end+.1):
                        f=min(frames,key=lambda f:abs(f['camera_read_monotonic']-enabled-desired))
                        timed.append({'image':(folder/'camera'/f['image']).relative_to(ROOT).as_posix(),
                                      't':f['camera_read_monotonic']-enabled,'phase':f['phase']})
                    print('STOP_TIMELINE',json.dumps(timed))
        ep=folder/'empty_recovery_screen/review.json'
        if ep.exists():
            er=json.loads(ep.read_bytes()); bad=[v for v in er['static_cases'] if not v['passed']]
            if bad:
                print('EMPTY_FAILURE_EXAMPLE',json.dumps(bad[0]))
