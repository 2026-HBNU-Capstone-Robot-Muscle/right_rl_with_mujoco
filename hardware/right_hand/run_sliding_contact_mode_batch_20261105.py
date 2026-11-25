"""Offline static contact-mode comparisons only."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,subprocess,sys,json
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip/sliding_workspace_grip';O.mkdir(parents=True,exist_ok=True)
ap=argparse.ArgumentParser();ap.add_argument('--mode',choices=('xpos','xneg','zpos','zneg'),required=True);ap.add_argument('--finger',choices=('middle','index'),default='middle');a=ap.parse_args()
for seed in ('current','open','raised'):
    log=O/f'{a.mode}_{seed}_{a.finger}.log'
    with log.open('w',encoding='utf-8') as f:code=subprocess.run([sys.executable,'-B',str(R/'propose_sliding_workspace_grip_20261105.py'),'--weight','1','--seed',seed,'--mode',a.mode,'--finger',a.finger],stdout=f,stderr=subprocess.STDOUT).returncode
    reportpath=O/f'weight1_{seed}_{a.mode}{"_index" if a.finger=="index" else ""}/review.json';d=json.loads(reportpath.read_bytes()) if reportpath.exists() else {}
    print(json.dumps(dict(mode=a.mode,finger=a.finger,seed=seed,exit_code=code,accepted=d.get('proposal_accepted'),finger_goals=d.get('grasp_finger_goal_ticks'),gap=d.get('minimum_upper_gap_mm'),contacts=d.get('contacts'))),flush=True)
    if code:raise SystemExit(code)
