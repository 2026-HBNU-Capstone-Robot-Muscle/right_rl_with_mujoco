"""A fresh empty prefix plus the separately reviewed middle probe suffix."""
import os,sys,json,argparse
os.environ['JAX_PLATFORMS']='cpu'
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
from pathlib import Path
from middle_probe_plan_20261106 import R,MiddleProbePlan,hashes
from middle_probe_gate_20261106 import make_review
def main():
    p=argparse.ArgumentParser();p.add_argument('request');p.add_argument('output');a=p.parse_args();output=Path(a.output).resolve();output.relative_to(R)
    if output.exists():raise RuntimeError('Refuse to replace frozen probe proof')
    plan=MiddleProbePlan(a.request);before=hashes();import screen_calibrated_empty_margin_20261028 as screen
    screen.OfflineRecovery=MiddleProbePlan;old=sys.argv;folder=output.parent/'empty_recovery_screen'
    try:sys.argv=['empty','--request',str(plan.SOURCE),'--output-folder',folder.relative_to(R).as_posix()];screen.main()
    finally:sys.argv=old
    result=make_review(plan,json.loads((folder/'review.json').read_bytes()),before);output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print('MIDDLE PROBE CURRENT REVIEW COMPLETE',flush=True)
if __name__=='__main__':main()
