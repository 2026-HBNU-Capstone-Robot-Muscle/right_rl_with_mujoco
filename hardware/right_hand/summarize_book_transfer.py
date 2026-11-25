"""Separate active-motion housing diagnostics from later human removal samples."""
import json
from pathlib import Path
from collections import defaultdict
root=Path(__file__).resolve().parent
folder=root/'book_dynamic_yaw_trial_20261024_222132_806567'
r=json.loads((folder/'transfer_diagnostic.json').read_bytes());phases=defaultdict(lambda:dict(samples=0,max_model_overlap_mm=0.))
for row in r['middle_ring_contact_samples']:
    p=phases[row['phase']];p['samples']+=1
    p['max_model_overlap_mm']=max(p['max_model_overlap_mm'],max(c['depth_mm'] for c in row['contacts']))
out=dict(phase_summary=dict(phases),native_primitive_self_checks_remain_enabled=True,
    additional_middle_ring_visual_housing_hull_checks_were_missing=True,
    convex_hull_overlap_is_not_measured_physical_depth=True,
    physical_middle_ring_contact_verified=False,hardware_commands_sent=0)
(folder/'housing_contact_phase_summary.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out))
