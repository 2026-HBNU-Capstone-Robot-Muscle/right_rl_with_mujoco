"""Fresh OFF recovery screen only; fixed combined evidence must already pass."""
import argparse,json,sys
from pathlib import Path
from pitch_slow_yaw_plan_20261029 import PitchSlowYawPlan,ROOT,hashes
from pitch_slow_yaw_gate_20261029 import make_review
def main():
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output');args=parser.parse_args()
    output=Path(args.output).resolve();output.relative_to(ROOT)
    if output.exists():raise ValueError('Refuse to overwrite review')
    plan=PitchSlowYawPlan(args.request);before=hashes();folder=output.parent/'empty_recovery_screen';folder.mkdir(exist_ok=True)
    import screen_pitch_slow_yaw_empty_20261029 as screen
    saved=sys.argv
    try:sys.argv=['empty','--request',str(plan.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];screen.screen.main()
    finally:sys.argv=saved
    review=make_review(plan,json.loads((folder/'review.json').read_bytes()),before)
    output.write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8');print('Combined review passed; no hardware writes',flush=True)
if __name__=='__main__':main()
