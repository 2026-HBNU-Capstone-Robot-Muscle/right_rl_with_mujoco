"""Fresh OFF empty recovery screen; offline only."""
import argparse,json,sys
from pathlib import Path
from nominal_cycle_plan_20261112 import ROOT,NominalCyclePlan,hashes
from nominal_cycle_gate_20261112 import make_review

p=argparse.ArgumentParser(); p.add_argument('request'); p.add_argument('output'); a=p.parse_args()
output=Path(a.output).resolve(); output.relative_to(ROOT)
if output.exists(): raise RuntimeError('Preserve proof')
plan=NominalCyclePlan(a.request); before=hashes()
import screen_calibrated_empty_margin_20261028 as screen
screen.OfflineRecovery=NominalCyclePlan
folder=output.parent/'empty_recovery_screen'
sys.argv=['empty','--request',str(plan.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()]
screen.main()
r=make_review(plan,json.loads((folder/'review.json').read_bytes()),before)
output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
print('Nominal cycle EMPTY review prepared; execution authorization is separate.',flush=True)
