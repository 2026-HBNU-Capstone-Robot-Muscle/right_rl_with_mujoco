"""Summarize the two saved user-run failures, without opening any device."""
import hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
folders=['grasp_approach_current_empty_20261022_015038_969787','grasp_approach_current_empty_20261022_015140_792784']
base=json.loads((ROOT/'prepared_placement_capture_20261022_010440_125130/result.json').read_bytes())
known=np.array([base['initial']['position'],base['placement_position_ticks'],base['placement_goal_ticks'],
               *[s['position'] for s in base['final_off_snapshots']]])
for name in folders:
    folder=ROOT/name;r=json.loads((folder/'result.json').read_bytes())
    samples=r.get('samples',[])
    summary={k:r.get(k) for k in ('timestamp','status','phase','failed_phase','error','all_torque_off_verified','settings_restored')}
    summary['source_sha256']=hashlib.sha256((folder/'result.json').read_bytes()).hexdigest()
    summary['top_level_keys']=list(r)
    summary['goal_write_attempts']=len([w for w in r.get('write_events',[]) if w['address']==116])
    summary['sample_count']=len(samples)
    if samples:
        summary.update(min_voltage=min(min(s['voltage_v']) for s in samples),
            peak_current=max(max(abs(i) for i in s['current_ma']) for s in samples),
            max_tracking=max(max(s['tracking_rad']) for s in samples))
        groups={}
        for s in samples:groups.setdefault(s['phase'],[]).append(s)
        summary['phases']={k:len(v) for k,v in groups.items()}
        summary['last_samples']=[{k:s[k] for k in ('phase','t','goal','actual','current_ma','tracking_rad')} for s in samples[-5:]]
        summary['last_on_position']=samples[-1]['actual'];summary['last_on_goal']=samples[-1]['goal']
    snapshots=r.get('final_off_snapshots',[])
    summary['off_positions']=[s['position'] for s in snapshots]
    summary['off_endpoint_drift']=r.get('final_off_endpoint_drift_ticks')
    if r.get('initial'):summary['initial_position']=r['initial']['position']
    pf=r.get('preflight_state')
    summary['preflight_state']=pf
    if isinstance(pf,dict) and 'position' in pf:
        vals=np.array(pf['position']);low=known.min(0)-32;high=known.max(0)+32
        summary['outside_old_admission']=[dict(id=i+1,current=int(vals[i]),lower=int(low[i]),upper=int(high[i]))
            for i in range(16) if not low[i]<=vals[i]<=high[i]]
    obs=r.get('off_start_observations',[])
    summary['off_acquisition_positions']=[s['position'] for s in obs]
    # preflight motors use the SDK-independent saved motor records.
    mot=r.get('preflight',[])
    summary['preflight_type']=type(mot).__name__
    if isinstance(mot,dict):summary['preflight_keys']=list(mot)
    if isinstance(mot,list):summary['preflight_excerpt']=mot[:1]
    frames=json.loads((folder/'camera/frames.json').read_bytes())
    groups={}
    for f in frames.get('frames',[]):groups.setdefault(f['phase'],[]).append(f)
    summary['phase_images']={k:dict(first=v[0]['image'],last=v[-1]['image'],count=len(v)) for k,v in groups.items()}
    (folder/'stop_review.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('top_level_keys','last_samples','preflight_excerpt','off_acquisition_positions','preflight_state')},ensure_ascii=False,indent=2))
    if samples:
        s=samples[-1];print('Last fixed errors:',[(i+1,s['actual'][i]-s['goal'][i],s['current_ma'][i]) for i in range(16) if abs(s['actual'][i]-s['goal'][i])>20])
    print('Keys:',list(r))
