import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
for d in sorted(ROOT.glob('quarter_support_photo_2026*'),reverse=True)[:3]:
    p=d/'result.json'
    if not p.exists():continue
    r=json.loads(p.read_bytes());print('RESULT',d.name)
    print(json.dumps({k:r.get(k) for k in ('status','phase','failed_phase','error','all_torque_off_verified','settings_restored','current_start_screen','camera_recording','camera_image_signal')},ensure_ascii=False))
    print('WRITES',len(r.get('write_events',[])))
    req=d/'fresh_off_request.json'
    if req.exists():print('POSITION',json.loads(req.read_bytes())['off_state']['position'])
    log=d/'current_start_screen.log'
    if log.exists():print('LOG_TAIL','\n'.join(log.read_text(encoding='utf-8',errors='replace').splitlines()[-35:]))
    screen=d/'current_start_screen.json'
    if screen.exists():
        s=json.loads(screen.read_bytes());print('SCREEN_KEYS',list(s));print('SCREEN_ERROR',s.get('error'))
