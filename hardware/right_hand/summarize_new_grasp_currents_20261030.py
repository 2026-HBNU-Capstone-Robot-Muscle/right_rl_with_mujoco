import json
from pathlib import Path
R=Path(__file__).resolve().parent
for folder in ('single_forward_trial_20261030_052636_068402','single_forward_trial_20261030_053131_966030'):
 r=json.loads((R/folder/'offline_review/forward_motion_review.json').read_bytes());print(folder)
 for phase,x in r['hold_stats'].items():print(phase,'current',x['median_current_ma'],'goal',x['goal'],'error',x['median_error_ticks'])
