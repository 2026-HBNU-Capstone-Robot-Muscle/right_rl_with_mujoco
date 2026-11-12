import argparse,json,sys
from pathlib import Path
from raised_support_capture_plan_20261104 import RaisedSupportCapturePlan,ROOT,hashes
from raised_support_capture_gate_20261104 import make_review
def main():
 ap=argparse.ArgumentParser();ap.add_argument('request');ap.add_argument('output');a=ap.parse_args();out=Path(a.output).resolve();out.relative_to(ROOT)
 if out.exists():raise RuntimeError('Do not overwrite a proof')
 p=RaisedSupportCapturePlan(a.request);before=hashes();import screen_calibrated_empty_margin_20261028 as s
 s.OfflineRecovery=RaisedSupportCapturePlan;argv=sys.argv;folder=out.parent/'empty_recovery_screen'
 try:sys.argv=['empty','--request',str(p.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];s.main()
 finally:sys.argv=argv
 r=make_review(p,json.loads((folder/'review.json').read_bytes()),before);out.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
