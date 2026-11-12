import argparse,json,sys
from pathlib import Path
from raised_cradle_plan_20261104 import RaisedCradlePlan,ROOT,hashes
from raised_cradle_gate_20261104 import make_review
def main():
 ap=argparse.ArgumentParser();ap.add_argument('request');ap.add_argument('output');args=ap.parse_args();out=Path(args.output).resolve();out.relative_to(ROOT)
 if out.exists():raise RuntimeError('Do not overwrite reviewed proof')
 p=RaisedCradlePlan(args.request);before=hashes();import screen_calibrated_empty_margin_20261028 as s
 s.OfflineRecovery=RaisedCradlePlan;previous=sys.argv;folder=out.parent/'empty_recovery_screen'
 try:sys.argv=['empty','--request',str(p.SOURCE),'--output-folder',folder.relative_to(ROOT).as_posix()];s.main()
 finally:sys.argv=previous
 result=make_review(p,json.loads((folder/'review.json').read_bytes()),before);out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print('RAISED CRADLE REVIEW COMPLETE',flush=True)
if __name__=='__main__':main()
