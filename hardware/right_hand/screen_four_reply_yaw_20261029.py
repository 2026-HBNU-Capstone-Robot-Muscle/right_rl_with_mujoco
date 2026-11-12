"""Same201static/9dynamic recovery and original loaded-route evidence."""
import argparse,json,sys
from pathlib import Path
from four_reply_yaw_plan_20261029 import FourReplyYawPlan,ROOT,hashes
from four_reply_yaw_gate_20261029 import make_review
def main():
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output');a=parser.parse_args()
    output=Path(a.output).resolve();output.relative_to(ROOT);folder=output.parent/'empty_recovery_screen';folder.mkdir(parents=True,exist_ok=True)
    if output.exists():raise ValueError('Refuse to overwrite existing review')
    p=FourReplyYawPlan(a.request);before=hashes()
    import screen_calibrated_empty_margin_20261028 as screen
    screen.OfflineRecovery=FourReplyYawPlan;saved=sys.argv
    try:sys.argv=['empty','--request',str(p.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];screen.main()
    finally:sys.argv=saved
    r=make_review(p,json.loads((folder/'review.json').read_bytes()),before);output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
