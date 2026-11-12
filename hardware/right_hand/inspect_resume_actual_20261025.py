"""Inspect newest saved EMPTY continuation reports without device access."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
for folder in sorted(ROOT.glob('reverse_quarter_resume_empty_20261025_*'))[-4:]:
    print('FOLDER',folder.name)
    result=folder/'result.json'
    if not result.exists():print('NO RESULT',sorted(p.name for p in folder.iterdir()));continue
    r=json.loads(result.read_bytes())
    print('RESULT',json.dumps({k:r.get(k) for k in ('status','phase','failed_phase','error','all_torque_off_verified','settings_restored','last_issued_command','planned_final_command','final_command','progress_fault','ring_release_measured_verified')},ensure_ascii=False))
    print('COUNTS',len(r.get('samples',[])),len(r.get('write_events',[])))
    samples=r.get('samples',[])
    if samples:
        print('SAMPLE_KEYS',list(samples[0]))
        print('LAST_SAMPLE',json.dumps(samples[-1],ensure_ascii=False))
        errs=np.array([np.abs(np.array(s['actual'])-s['goal'])*2*np.pi/4096 for s in samples])
        print('MAX_TRACKING',float(errs.max()),'DURATION',samples[-1]['t'])
        print('PHASE_COUNTS',{p:sum(s['phase']==p for s in samples) for p in dict.fromkeys(s['phase'] for s in samples)})
    for name in ('current_start_screen.log','current_start_screen.json'):
        path=folder/name
        if path.exists():
            if path.suffix=='.log':print('SCREEN_LOG',path.read_text(encoding='utf-8',errors='replace')[-4000:])
            else:
                s=json.loads(path.read_bytes());print('SCREEN',s.get('accepted_for_supervised_empty_preparation'),len(s.get('static_screens',[])),len(s.get('dynamic_sensitivity',[])))
    print('FILES',sorted(p.name for p in folder.iterdir()))
for parent in [ROOT,*ROOT.parents]:
    f=parent/'AGENTS.md'
    if f.exists():print('AGENTS',str(f),f.read_text(encoding='utf-8'))
