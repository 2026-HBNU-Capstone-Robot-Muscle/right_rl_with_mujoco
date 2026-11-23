"""Fresh EMPTY approach only; loaded route has its own immutable proof."""
import sys,json
from pathlib import Path
import screen_final_pose_recovery_20261116 as screen
from cube_grasp_hold_20261116 import Plan,make_review
def main():
    request,output=map(Path,sys.argv[1:]);screen.OfflineRecovery=Plan
    folder=output.parent/'cube_empty_recovery';old=sys.argv
    try:
        sys.argv=['screen','--request',str(request),'--output-folder',str(folder)];screen.main()
    finally:sys.argv=old
    r=make_review(Plan(request),json.loads((folder/'review.json').read_bytes()))
    output.write_text(json.dumps(r,indent=2)+'\n')
if __name__=='__main__':main()
