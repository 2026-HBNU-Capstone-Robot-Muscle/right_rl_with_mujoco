"""Compare exact matching saved grasp goals with an existing empty reference."""
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_grasp_trial_20261023_044607_477240'

def main():
    latest=json.loads((FOLDER/'result.json').read_bytes())
    empty=json.loads((ROOT/'supported_yaw_empty_20261023_030552_675209/result.json').read_bytes())
    a=[s for s in latest['samples'] if s['phase']=='stage_3_loaded_approach']
    b=[s for s in empty['samples'] if s['phase']=='stage_3_grasp_rehearsal_EMPTY']
    assert len(a)==len(b)==322
    assert np.array_equal([s['goal'] for s in a],[s['goal'] for s in b])
    rows=[]
    for ident in range(1,17):
        i=ident-1
        ac=np.array([s['current_ma'][i] for s in a[-30:]])
        bc=np.array([s['current_ma'][i] for s in b[-30:]])
        ae=np.array([s['goal'][i]-s['actual'][i] for s in a[-30:]])
        be=np.array([s['goal'][i]-s['actual'][i] for s in b[-30:]])
        rows.append(dict(motor_id=ident,current_loaded_ma=float(np.median(ac)),
            current_empty_ma=float(np.median(bc)),extra_signed_current_ma=float(np.median(ac-bc)),
            signed_error_loaded_ticks=float(np.median(ae)),signed_error_empty_ticks=float(np.median(be)),
            extra_signed_error_ticks=float(np.median(ae-be))))
    result=dict(source=FOLDER.name,status='exact_goal_load_comparison',matching_goal_samples=322,
        tail_samples=30,rows=rows,
        maximum_absolute_added_median_current_ma=max(abs(r['extra_signed_current_ma']) for r in rows),
        limitation='Separate runs: voltages/history/friction may differ. Current is not calibrated contact force; a small difference does not prove no contact.',
        physical_grasp_verified=False,hardware_commands_sent=0)
    (FOLDER/'matched_empty_load_review.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
