"""Fresh actual-OFF empty recovery screen; no physical device APIs."""
import argparse
import json
import sys
from pathlib import Path
from support_first_trial_plan_20261112 import ROOT,SupportFirstTrialPlan,hashes
from support_first_trial_gate_20261112 import make_review

def main():
    p=argparse.ArgumentParser();p.add_argument('request');p.add_argument('output');a=p.parse_args()
    output=Path(a.output).resolve();output.relative_to(ROOT)
    if output.exists():raise RuntimeError('Preserve current-pose proof')
    plan=SupportFirstTrialPlan(a.request);before=hashes()
    import screen_calibrated_empty_margin_20261028 as screen
    screen.OfflineRecovery=SupportFirstTrialPlan;folder=output.parent/'empty_recovery_screen';old=sys.argv
    try:
        sys.argv=['empty','--request',str(plan.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];screen.main()
    finally:sys.argv=old
    r=make_review(plan,json.loads((folder/'review.json').read_bytes()),before)
    output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');print('Support-first fresh EMPTY review complete.',flush=True)

if __name__=='__main__':main()
