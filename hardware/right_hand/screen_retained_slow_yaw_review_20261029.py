"""Offline checker used only before any user-started torque enabling."""
import argparse,json,sys
from pathlib import Path
from retained_slow_yaw_plan_20261029 import RetainedSlowYawPlan,ROOT,hashes
from retained_slow_yaw_gate_20261029 import make_review
def main():
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output');a=parser.parse_args()
    output=Path(a.output).resolve();output.relative_to(ROOT);folder=output.parent/'empty_recovery_screen';folder.mkdir(parents=True,exist_ok=True)
    if output.exists():raise ValueError('Refuse to overwrite existing review')
    plan=RetainedSlowYawPlan(a.request);before=hashes()
    import screen_retained_slow_yaw_empty_20261029 as screen
    saved=sys.argv
    try:sys.argv=['empty','--request',str(plan.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];screen.screen.main()
    finally:sys.argv=saved
    r=make_review(plan,json.loads((folder/'review.json').read_bytes()),before)
    output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
