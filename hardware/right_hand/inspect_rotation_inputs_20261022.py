"""Compact offline inventory for the first rotation search."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'prepared_cube_review_20261022'

if __name__=='__main__':
    r=json.loads((ROOT/'cube_grasp_trial_20261022_141534_666300/result.json').read_bytes())
    print('FINAL ON', {k:r['final_on_state'][k] for k in ('position','goal')})
    for name in ('loaded_approach_sensitivity.json','support_settling_scan.json','placement_review.json'):
        d=json.loads((OUT/name).read_bytes());print(name,'keys',list(d))
        for k,v in d.items():
            if isinstance(v,list):
                print(k,'length',len(v))
                for row in v[:3]:
                    if isinstance(row,dict):
                        print({a:b for a,b in row.items() if a not in ('trace','trajectory','frames')})
            else: print(k,str(v)[:1200])
    print('RELEVANT FILES')
    for pattern in ('*loaded*py','*tilt*py','*teacher*py','*sac*train*py','*reference*py'):
        print(pattern,[p.name for p in ROOT.glob(pattern)])
