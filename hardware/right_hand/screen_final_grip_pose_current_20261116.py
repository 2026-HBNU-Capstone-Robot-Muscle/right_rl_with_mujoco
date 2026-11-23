"""Check both fresh OFF recovery and the full empty final-pose path, OFF only."""
import argparse
import importlib
import json
import sys
from pathlib import Path
from final_grip_pose_plan_20261116 import ROOT, FinalGripPosePlan, hashes
from final_grip_pose_gate_20261116 import make_review

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('request'); parser.add_argument('output')
    args=parser.parse_args()
    output=Path(args.output).resolve(); output.relative_to(ROOT)
    if output.exists():
        raise RuntimeError('Preserve existing pose proof')
    plan=FinalGripPosePlan(args.request); before=hashes(); reports=[]
    for name, folder in [('screen_final_pose_recovery_20261116','empty_recovery_screen'),
                         ('screen_final_grip_pose_full_20261116','full_pose_screen')]:
        screen=importlib.import_module(name); screen.OfflineRecovery=FinalGripPosePlan
        dest=output.parent/folder; previous=sys.argv
        try:
            sys.argv=['screen','--request',str(plan.SOURCE),'--output-folder',dest.relative_to(ROOT).as_posix()]
            screen.main()
        finally:
            sys.argv=previous
        reports.append(json.loads((dest/'review.json').read_bytes()))
    result=make_review(plan,*reports,before)
    output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('Complete empty final-pose review passed. No devices opened.',flush=True)

if __name__=='__main__':
    main()
