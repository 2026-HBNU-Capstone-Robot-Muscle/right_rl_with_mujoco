"""Fresh EMPTY recovery only. Loaded models stay source bound separately."""
import argparse,json,sys
from pathlib import Path
from natural_cradle_grasp_plan_20261030 import NaturalCradleGraspPlan,ROOT,hashes
from natural_cradle_grasp_gate_20261030 import make_review
def main():
 ap=argparse.ArgumentParser();ap.add_argument('request');ap.add_argument('output');a=ap.parse_args();out=Path(a.output).resolve();out.relative_to(ROOT)
 if out.exists():raise RuntimeError('Refuse to overwrite natural-grasp proof')
 p=NaturalCradleGraspPlan(a.request);before=hashes();import screen_calibrated_empty_margin_20261028 as s
 s.OfflineRecovery=NaturalCradleGraspPlan;old=sys.argv;folder=out.parent/'empty_recovery_screen'
 try:sys.argv=['empty','--request',str(p.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];s.main()
 finally:sys.argv=old
 r=make_review(p,json.loads((folder/'review.json').read_bytes()),before);out.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');print('NATURAL GRASP REVIEW COMPLETE',flush=True)
if __name__=='__main__':main()
