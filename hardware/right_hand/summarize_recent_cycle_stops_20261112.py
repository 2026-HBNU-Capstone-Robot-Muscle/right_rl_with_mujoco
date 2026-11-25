"""Summarize only recent saved startup rejections; no device access."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
for stamp in ('060336_883634','060515_800377','061720_638673'):
    folder=ROOT/('nominal_two_turn_trial_20261112_'+stamp)
    path=folder/'empty_recovery_screen/review.json'
    if not path.exists():
        print(stamp,'no saved empty review');continue
    r=json.loads(path.read_bytes());bad=[x for x in r['static_cases'] if not x['passed']]
    dynamic=[x for x in r['dynamic_cases'] if not x['passed']]
    pairs={}
    for row in bad:
        for pair,value in row.get('worst_self_pairs_mm',{}).items():pairs[pair]=max(pairs.get(pair,0),value)
    print(json.dumps({'run':stamp,'static_failed':len(bad),'dynamic_failed':len(dynamic),
                      'self_pairs_mm':pairs,'example_dynamic':{k:v for k,v in (dynamic[0] if dynamic else {}).items()
                        if k in ('max_self_mm','max_support_mm','max_tracking_rad','unapproved_range_excess_rad','initial_support_increase_mm','root_clear_seconds','initial_pair_scope_valid')}},ensure_ascii=False))
