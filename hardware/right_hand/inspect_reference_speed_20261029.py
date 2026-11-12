import json,numpy as np
from pathlib import Path
R=Path(__file__).resolve().parent
for name in ('calibrated_quarter_trial_20261028_174723_390729','paced_retained_yaw_20261029_044827_659675'):
    r=json.loads((R/name/'result.json').read_bytes());s=[x for x in r['samples'] if x['phase']=='stage_7_horizontal_yaw'];d=[]
    for a,b in zip(s,s[1:]):
        if a['goal']!=b['goal']:d.append(b['t']-a['t'])
    print(json.dumps(dict(run=name,status=r['status'],period=r.get('requested_sample_period_s'),yaw_samples=len(s),
        minimum_distinct_goal_dt=float(min(d)),median_adjacent_dt=float(np.median([b['t']-a['t'] for a,b in zip(s,s[1:])])),
        gain_config={k:r['initial'][k][11] for k in ('p_gain','d_gain','current_cap')},yaw_root_peak_current=max(abs(x['current_ma'][11]) for x in s))))
for name in ('right_hand_rotation_20261028/observed90_prefix638/full_loaded_27/review.json',
    'right_hand_rotation_20261028/observed90_prefix638/static_201/review.json'):
    r=json.loads((R/name).read_bytes());print(name,json.dumps({k:len(v) if isinstance(v,(list,dict)) else v for k,v in r.items()}))
