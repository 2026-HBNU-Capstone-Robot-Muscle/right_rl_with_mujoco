"""Saved reports only; no hardware access."""
import json
from pathlib import Path
root=Path(__file__).resolve().parent
for folder in sorted(root.glob('loaded_quarter_trial_20261025_*'))[-4:]:
    p=folder/'result.json'
    if not p.exists():continue
    r=json.loads(p.read_bytes());print('FOLDER',folder.name)
    print('RESULT',json.dumps({k:r.get(k) for k in ('status','phase','failed_phase','error','all_torque_off_verified','settings_restored','cube_placement_confirmed','cube_removal_confirmed','last_issued_command','progress_fault','ring_release_measured_verified','thumb_tip_release_measured_verified')},ensure_ascii=False))
    print('WRITES',len(r.get('write_events',[])),'SAMPLES',len(r.get('samples',[])))
    if r.get('samples'):print('LAST_SAMPLE',json.dumps(r['samples'][-1],ensure_ascii=False))
    req=folder/'fresh_off_request.json'
    if req.exists():
        s=json.loads(req.read_bytes());print('START',s['off_state']['position'])
    log=folder/'current_start_screen.log'
    if log.exists():print('LOG_TAIL',log.read_text(encoding='utf-8',errors='replace')[-3500:])
    print('FILES',[p.name for p in folder.iterdir()])
