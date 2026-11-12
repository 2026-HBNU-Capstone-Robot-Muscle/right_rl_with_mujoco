"""Matched-goal load diagnostics from completed stored runs, no devices."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
folder=ROOT/'cube_supported_rotation_trial_20261023_034640_750784'
loaded=json.loads((folder/'result.json').read_bytes())
empty=json.loads((ROOT/'supported_yaw_empty_20261023_030552_675209/result.json').read_bytes())
a=[s for s in loaded['samples'] if s['phase']=='stage_3_loaded_approach']
b=[s for s in empty['samples'] if s['phase']=='stage_3_grasp_rehearsal_EMPTY']
assert len(a)==len(b)==322
assert np.array_equal([s['goal'] for s in a],[s['goal'] for s in b])
rows=[]
for ident in range(1,17):
    i=ident-1
    ca=np.array([s['current_ma'][i] for s in a[-30:]])
    cb=np.array([s['current_ma'][i] for s in b[-30:]])
    ea=np.array([s['goal'][i]-s['actual'][i] for s in a[-30:]])
    eb=np.array([s['goal'][i]-s['actual'][i] for s in b[-30:]])
    row=dict(motor_id=ident,loaded_current_ma=float(np.median(ca)),empty_current_ma=float(np.median(cb)),
        added_signed_current_ma=float(np.median(ca-cb)),
        loaded_signed_error_ticks=float(np.median(ea)),empty_signed_error_ticks=float(np.median(eb)),
        added_signed_error_ticks=float(np.median(ea-eb)),
        added_current_q10_q90_ma=np.quantile(ca-cb,[.1,.9]).tolist())
    rows.append(row)
out=dict(status='matched_grasp_goal_empty_vs_loaded_diagnostic',goal_samples_identical=322,
    tail_samples_compared=30,rows=rows,physical_contact_verified=False,
    limitations=['Separate runs; motor temperature, voltage, motion history and contact conditions can differ.',
                 'Differences are diagnostic load evidence, not calibrated force or proof of absence of contact.'])
(folder/'empty_vs_loaded_grasp.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
for r in rows:print(r['motor_id'],'current loaded/empty/delta',r['loaded_current_ma'],r['empty_current_ma'],r['added_signed_current_ma'],
    'error loaded/empty/delta',r['loaded_signed_error_ticks'],r['empty_signed_error_ticks'],r['added_signed_error_ticks'])
