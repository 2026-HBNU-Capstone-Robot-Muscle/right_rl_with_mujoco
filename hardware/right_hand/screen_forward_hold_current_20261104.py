"""Current EMPTY checks and evidence assembly; device-free."""
import argparse,json,sys,os
os.environ['JAX_PLATFORMS']='cpu'
from pathlib import Path
from forward_hold_plan_20261104 import ROOT,ForwardHoldPlan,hashes
from forward_hold_gate_20261104 import make_review
def main():
    ap=argparse.ArgumentParser();ap.add_argument('request');ap.add_argument('output');a=ap.parse_args();out=Path(a.output).resolve();out.relative_to(ROOT)
    if out.exists():raise RuntimeError('Refuse to overwrite proof')
    p=ForwardHoldPlan(a.request);before=hashes();import screen_calibrated_empty_margin_20261028 as s
    s.OfflineRecovery=ForwardHoldPlan;saved=sys.argv;folder=out.parent/'empty_recovery_screen'
    try:sys.argv=['empty','--request',str(p.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];s.main()
    finally:sys.argv=saved
    r=make_review(p,json.loads((folder/'review.json').read_bytes()),before);out.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');print('FORWARD HOLD REVIEW COMPLETE',flush=True)
if __name__=='__main__':main()
