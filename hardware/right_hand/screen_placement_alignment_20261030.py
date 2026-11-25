"""Only the current EMPTY prefix is screened. No devices."""
import argparse,json,sys
from pathlib import Path
from placement_alignment_plan_20261030 import PlacementAlignmentPlan,ROOT,hashes
from placement_alignment_gate_20261030 import make_review
def main():
 ap=argparse.ArgumentParser();ap.add_argument('request');ap.add_argument('output');a=ap.parse_args();out=Path(a.output).resolve();out.relative_to(ROOT)
 if out.exists():raise RuntimeError('Refuse to overwrite an alignment proof')
 p=PlacementAlignmentPlan(a.request);before=hashes();import screen_calibrated_empty_margin_20261028 as s
 s.OfflineRecovery=PlacementAlignmentPlan;old=sys.argv;folder=out.parent/'empty_recovery_screen'
 try:sys.argv=['empty','--request',str(p.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];s.main()
 finally:sys.argv=old
 r=make_review(p,json.loads((folder/'review.json').read_bytes()),before);out.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');print('PLACEMENT-ONLY REVIEW COMPLETE',flush=True)
if __name__=='__main__':main()
