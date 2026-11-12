"""Read saved trial reports and the user's pasted log; no device access."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
attachment=Path('C:/Users/김준서/.codex/attachments/ba0a4676-0162-4d9a-8aa0-789213a058b5/Pasted text.txt')
if attachment.exists():
    print('PASTED_LOG')
    print(attachment.read_text(encoding='utf-8-sig'))
for folder in sorted(ROOT.glob('reverse_quarter_path_empty_20261025_*'))[-3:]:
    r=json.loads((folder/'result.json').read_bytes())
    print('RESULT',folder.name,json.dumps({k:r.get(k) for k in ('status','phase','error','failed_phase','all_torque_off_verified','settings_restored','enabled_at','final_command')},ensure_ascii=False))
    print('WRITES',len(r.get('write_events',[])),'SAMPLES',len(r.get('samples',[])))
    for sample in r.get('samples',[])[-3:]:
        print('LAST_SAMPLE',json.dumps(sample,ensure_ascii=False))
