"""Run only the selected offline reaction evaluator, with per-run logs."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,subprocess,sys,json
from pathlib import Path
R=Path(__file__).resolve().parent
O=R/'right_hand_rotation_20261105/canonical_body_grip'
ap=argparse.ArgumentParser();ap.add_argument('--friction',type=float,choices=(.2,.3,.5),required=True);a=ap.parse_args()
candidate=O/'tip_squeeze/id1/plus12/candidate.npz';logdir=O/'layer_reaction/v3_batch_logs';logdir.mkdir(parents=True,exist_ok=True)
for gain in (.5,1.,1.5):
    args=[sys.executable,'-B',str(R/'evaluate_canonical_grip_reaction_20261105.py'),'--friction',str(a.friction),'--gain',str(gain),'--candidate',str(candidate),'--matrix-tag','full_placement_v3_thumb12','--output-tag','selected_v3_thumb12','--magnitude','.01']
    log=logdir/f'g{gain:g}_f{a.friction:g}.log'
    with log.open('w',encoding='utf-8') as f:code=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT).returncode
    print(json.dumps(dict(gain=gain,friction=a.friction,exit_code=code,log=str(log.relative_to(R)))),flush=True)
    if code:raise SystemExit(code)
