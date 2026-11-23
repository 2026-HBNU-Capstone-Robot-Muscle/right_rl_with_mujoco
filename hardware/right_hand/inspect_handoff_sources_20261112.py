"""Small inventory for a portable teammate handoff and meeting brief."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
for pattern in ['*professor*','*meeting*','*requirements*','*handoff*','*policy*status*']:
    print(pattern,[p.name for p in ROOT.glob(pattern) if p.is_file()][:25])
for name in ['README.md','CURRENT_STATE.md','right_hand_rotation_20261109/policy_audit/STATUS.md','cloud/setup.sh','cloud/SETUP.md','github_desktop/DESKTOP_UPLOAD_COMPLETE.json']:
    p=ROOT/name
    print('\nFILE',name,'exists',p.exists())
    if p.exists() and p.is_file():print(p.read_text(encoding='utf-8-sig')[:6000])
p=ROOT/'right_hand_rotation_20261112/tip_first_empty_initial_scope/trace.json'
if p.exists():print('TRACE_SUMMARY',json.dumps(json.loads(p.read_bytes())['summary'],ensure_ascii=False))
print('LOWER_REVIEW_IMAGES',[(p.name,p.stat().st_size) for p in (ROOT/'right_hand_rotation_20261112/lower_grip_actual_review').glob('*.jpg')])
