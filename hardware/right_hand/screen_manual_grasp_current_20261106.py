"""Screen only the current empty recovery; preserve the executed loaded prefix."""
import os,sys,json,argparse
os.environ['JAX_PLATFORMS']='cpu'
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
from pathlib import Path
from manual_grasp_plan_20261106 import R,ManualGraspPlan,hashes
from manual_grasp_gate_20261106 import make_review
def main():
    p=argparse.ArgumentParser();p.add_argument('request');p.add_argument('output');a=p.parse_args();output=Path(a.output).resolve();output.relative_to(R)
    if output.exists():raise RuntimeError('Refuse to replace a proof')
    plan=ManualGraspPlan(a.request);before=hashes();import screen_calibrated_empty_margin_20261028 as screen
    screen.OfflineRecovery=ManualGraspPlan;old=sys.argv;folder=output.parent/'empty_recovery_screen'
    try:sys.argv=['empty','--request',str(plan.SOURCE),'--output-folder',folder.relative_to(R).as_posix()];screen.main()
    finally:sys.argv=old
    result=make_review(plan,json.loads((folder/'review.json').read_bytes()),before);output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print('MANUAL TEACH MOTION REVIEW COMPLETE',flush=True)
if __name__=='__main__':main()
