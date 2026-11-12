"""Fresh OFF setup screen; fixed loaded evidence is independently hash checked."""
import argparse,json,sys
from pathlib import Path
from pitch_handoff_plan_20261028 import PitchHandoffPlan,ROOT,hashes
from pitch_handoff_gate_20261028 import make_review

def main():
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output');args=parser.parse_args()
    output=Path(args.output).resolve();output.relative_to(ROOT)
    if output.exists():raise ValueError('Refuse an existing review output')
    plan=PitchHandoffPlan(args.request);before=hashes()
    folder=output.parent/'empty_recovery_screen';folder.mkdir(exist_ok=True)
    import screen_pitch_handoff_empty_20261028 as empty
    saved=sys.argv
    try:
        sys.argv=['empty','--request',str(Path(args.request).resolve()),'--output-folder',folder.relative_to(ROOT).as_posix()]
        empty.screen.main()
    finally:sys.argv=saved
    result=make_review(plan,json.loads((folder/'review.json').read_bytes()),before)
    output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('PITCH/TRANSFER OBSERVATION REVIEW PASSED; no devices opened',flush=True)

if __name__=='__main__':main()
