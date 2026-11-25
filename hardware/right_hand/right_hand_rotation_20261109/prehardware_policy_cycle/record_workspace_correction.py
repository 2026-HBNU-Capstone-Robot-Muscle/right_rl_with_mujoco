"""Preserve and withdraw a native-distance diagnostic before replacing it."""
from pathlib import Path
import json
import hashlib
import numpy as np

OUT=Path(__file__).resolve().parent
source=(OUT/'audit_regrasp_workspace.py').read_bytes()
for finger in ('rf','mf'):
    folder=OUT/'next_cycle'/('workspace_'+finger)
    report=json.loads((folder/'review.json').read_text())
    if report.get('native_distance_diagnostic_withdrawn'):continue
    (folder/'source_before_distance_correction.py').write_bytes(source)
    report['native_distance_diagnostic_withdrawn']=True
    report['not_valid_for_kinematic_reach_claims']=True
    report['correction_reason']='Some native distances are zero while returned witness endpoints remain far apart. Replace with existing independent convex separation checks.'
    report['preserved_source_sha256']=hashlib.sha256(source).hexdigest()
    for case in report['cases']:
        witness=np.asarray(case['closest_segment_world'])
        case['witness_endpoint_distance_mm']=float(np.linalg.norm(witness[3:]-witness[:3])*1000)
        case['original_pure_kinematic_contact_found']=case.pop('pure_kinematic_contact_found')
        case['kinematic_contact_not_certified']=True
    (folder/'review.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({'native_distance_reports_withdrawn':2,'hardware_execution_allowed':False}))
