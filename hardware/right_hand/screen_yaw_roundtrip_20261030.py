"""Current OFF recovery review, with no motor/camera access."""
import argparse,json,sys
from pathlib import Path
from yaw_roundtrip_plan_20261030 import YawRoundtripPlan,ROOT,hashes
from yaw_roundtrip_gate_20261030 import make_review
def main():
    ap=argparse.ArgumentParser();ap.add_argument('request');ap.add_argument('output');a=ap.parse_args()
    output=Path(a.output).resolve();output.relative_to(ROOT);folder=output.parent/'empty_recovery_screen';folder.mkdir(parents=True,exist_ok=True)
    if output.exists():raise RuntimeError('Refuse to overwrite roundtrip proof')
    p=YawRoundtripPlan(a.request);before=hashes();import screen_calibrated_empty_margin_20261028 as s
    s.OfflineRecovery=YawRoundtripPlan;saved=sys.argv
    try:sys.argv=['empty','--request',str(p.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];s.main()
    finally:sys.argv=saved
    review=make_review(p,json.loads((folder/'review.json').read_bytes()),before)
    output.write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8');print('CONTINUOUS ROUNDTRIP REVIEW COMPLETE',flush=True)
if __name__=='__main__':main()
