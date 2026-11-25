import argparse,json,sys
from pathlib import Path
from empty_hold_comm_plan_20261029 import ROOT,EmptyHoldCommPlan,hashes
from empty_hold_comm_gate_20261029 import make_review
def main():
    p=argparse.ArgumentParser();p.add_argument('request');p.add_argument('output');a=p.parse_args()
    output=Path(a.output).resolve();output.relative_to(ROOT)
    if output.exists():raise RuntimeError('Refuse existing output')
    plan=EmptyHoldCommPlan(a.request);before=hashes();folder=output.parent/'empty_hold_screen';folder.mkdir(exist_ok=True)
    import screen_empty_hold_comm_20261029 as screen
    saved=sys.argv
    try:sys.argv=['empty','--request',str(plan.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];screen.main()
    finally:sys.argv=saved
    r=make_review(plan,json.loads((folder/'review.json').read_bytes()),before);output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
