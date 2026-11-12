"""Compact local state inspection, never serial/camera."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
for name in ('sac_current_status.md','professor_progress_20261015.md'):
    print('STATUS',name)
    print('\n'.join((ROOT/name).read_text(encoding='utf-8').splitlines()[:36]))
folder=ROOT/'grasped_yaw_trial_20261023_234707_886202'
for name in ('completed_yaw_review.json','cube_plane_optical_flow.json'):
    r=json.loads((folder/name).read_bytes())
    print('REVIEW',name,{k:v for k,v in r.items() if k not in ('matches','points','source_frames','camera_recording')})
print('RECENT_TRIALS',[p.name for p in sorted(ROOT.glob('grasped_yaw_trial_20*'))[-5:]])
