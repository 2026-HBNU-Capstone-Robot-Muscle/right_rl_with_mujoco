"""Concise saved capture diagnosis; no device access."""
import json
from pathlib import Path
from collections import Counter
root=Path(__file__).resolve().parent
run=json.loads((root/'leap_sac_start_capture_result.json').read_text(encoding='utf-8'))
print({k:run.get(k) for k in ('timestamp','status','error','all_torque_off_verified','settings_restored')})
vision=json.loads((Path(run['vision_recording']['folder'])/'result.json').read_text(encoding='utf-8'))
for phase in ('preflight','stage_0','stage_2','revised_start_empty_reference'):
    rows=[r for r in vision['frames'] if r['phase']==phase]
    print(phase,len(rows),dict(Counter(tuple(r.get('anchor_detected_ids',[])) for r in rows)))
