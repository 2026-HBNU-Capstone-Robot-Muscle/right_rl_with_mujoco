"""Fresh recovery only; reuse a hash-identical fixed OPEN/grasp/yaw/release."""
import argparse,json,sys
from pathlib import Path
from calibrated_quarter_plan_20261028 import HorizontalQuarterPlan,ROOT,hashes
from calibrated_quarter_gate_20261028 import make_review

def main():
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output');args=parser.parse_args();output=Path(args.output).resolve();output.relative_to(ROOT)
    if output.exists():raise ValueError('Refuse to reuse an existing review')
    plan=HorizontalQuarterPlan(args.request);before=hashes();folder=output.parent/'empty_recovery_screen';folder.mkdir(exist_ok=True)
    import screen_horizontal_empty_recovery_20261027 as empty
    old=sys.argv
    try:
        sys.argv=['empty','--request',str(Path(args.request).resolve()),'--output-folder',folder.relative_to(ROOT).as_posix()];empty.main()
    finally:sys.argv=old
    result=make_review(plan,json.loads((folder/'review.json').read_bytes()),before);output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print('CALIBRATION ROUTE REVIEW PASSED; no robot writes',flush=True)
if __name__=='__main__':main()
