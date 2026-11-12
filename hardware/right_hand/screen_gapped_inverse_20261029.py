"""Fresh OFF prefix review; never opens a camera or motor port."""
import argparse,json,sys
from pathlib import Path
from gapped_inverse_plan_20261029 import GappedInversePlan,ROOT,hashes
from gapped_inverse_gate_20261029 import make_review
def main():
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output');a=parser.parse_args()
    output=Path(a.output).resolve();output.relative_to(ROOT)
    folder=output.parent/'empty_recovery_screen';folder.mkdir(parents=True,exist_ok=True)
    if output.exists():raise RuntimeError('Refuse to overwrite inverse proof')
    p=GappedInversePlan(a.request);before=hashes()
    import screen_calibrated_empty_margin_20261028 as screen
    screen.OfflineRecovery=GappedInversePlan;saved=sys.argv
    try:sys.argv=['empty','--request',str(p.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];screen.main()
    finally:sys.argv=saved
    review=make_review(p,json.loads((folder/'review.json').read_bytes()),before)
    output.write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
    print('GAPPED INVERSE REVIEW COMPLETE',flush=True)
if __name__=='__main__':main()
