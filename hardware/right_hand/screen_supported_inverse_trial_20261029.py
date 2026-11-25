"""Current OFF-to-OPEN prefix review for the supported cube observation."""
import argparse,json,sys
from pathlib import Path
from supported_inverse_trial_plan_20261029 import SupportedInverseTrialPlan,ROOT,hashes
from supported_inverse_trial_gate_20261029 import make_review
def main():
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output');args=parser.parse_args()
    output=Path(args.output).resolve();output.relative_to(ROOT)
    if output.exists():raise RuntimeError('Refuse to overwrite cube proof')
    folder=output.parent/'empty_recovery_screen';folder.mkdir(parents=True,exist_ok=True)
    p=SupportedInverseTrialPlan(args.request);before=hashes()
    import screen_calibrated_empty_margin_20261028 as screen
    screen.OfflineRecovery=SupportedInverseTrialPlan;saved=sys.argv
    try:sys.argv=['prefix','--request',str(p.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];screen.main()
    finally:sys.argv=saved
    result=make_review(p,json.loads((folder/'review.json').read_bytes()),before)
    output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print('SUPPORTED CUBE OBSERVATION REVIEW COMPLETE',flush=True)
if __name__=='__main__':main()
