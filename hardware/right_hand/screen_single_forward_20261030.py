"""Current empty recovery and exact single-forward evidence assembly. No devices."""
import argparse,json,sys
from pathlib import Path
from single_forward_plan_20261030 import SingleForwardPlan,ROOT,hashes
from single_forward_gate_20261030 import make_review
def main():
 ap=argparse.ArgumentParser();ap.add_argument('request');ap.add_argument('output');a=ap.parse_args();out=Path(a.output).resolve();out.relative_to(ROOT)
 if out.exists():raise RuntimeError('Refuse to overwrite single-forward proof')
 p=SingleForwardPlan(a.request);before=hashes();import screen_calibrated_empty_margin_20261028 as s
 s.OfflineRecovery=SingleForwardPlan;saved=sys.argv;folder=out.parent/'empty_recovery_screen'
 try:sys.argv=['empty','--request',str(p.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];s.main()
 finally:sys.argv=saved
 review=make_review(p,json.loads((folder/'review.json').read_bytes()),before);out.write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8');print('SINGLE FORWARD REVIEW COMPLETE',flush=True)
if __name__=='__main__':main()
