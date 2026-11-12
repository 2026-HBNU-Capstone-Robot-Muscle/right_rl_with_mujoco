"""Offline-only empty-path and placement-offset followups, saved per stage."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,subprocess,sys
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip'
ap=argparse.ArgumentParser();ap.add_argument('--front',type=int,choices=(-15,-10,-5),required=True);a=ap.parse_args()
candidate=O/'tip_squeeze/id1/plus12/candidate.npz';logs=O/'v3_followup_logs';logs.mkdir(parents=True,exist_ok=True)
common=['--candidate',str(candidate),'--matrix-tag','full_placement_v3_thumb12']
jobs=[('empty',['screen_canonical_grip_empty_20261105.py','--front',str(a.front),'--output-tag','empty_path_v3_thumb12']+common),('placement',['screen_canonical_grip_placement_offsets_20261105.py','--forward-mm',str({-15:-2,-10:0,-5:2}[a.front]),'--output-tag','placement_offsets_v3_thumb12']+common)]
for stage,args in jobs:
    log=logs/f'front{a.front}_{stage}.log'
    with log.open('w',encoding='utf-8') as f:code=subprocess.run([sys.executable,'-B',str(R/args[0])]+args[1:],stdout=f,stderr=subprocess.STDOUT).returncode
    print(json.dumps(dict(stage=stage,front=a.front,exit_code=code,log=str(log.relative_to(R)))),flush=True)
    if code:raise SystemExit(code)
